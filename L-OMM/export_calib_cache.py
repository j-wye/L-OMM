import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np
import os
import glob
import argparse
import json
import random
from PIL import Image
from transformers import Owlv2Processor, OwlViTProcessor
from typing import Tuple

NORMALIZATION_MEAN = np.array([0.48145466 * 255., 0.4578275 * 255., 0.40821073 * 255.], dtype=np.float32)
NORMALIZATION_STD = np.array([0.26862954 * 255., 0.26130258 * 255., 0.27577711 * 255.], dtype=np.float32)
MODEL_PROPERTIES = {
    "google/owlv2-base-patch16": {"image_size": 960, "patch_size": 16, "processor": Owlv2Processor, "max_text_length": 16},
    "google/owlv2-base-patch16-ensemble": {"image_size": 960, "patch_size": 16, "processor": Owlv2Processor, "max_text_length": 16},
    "google/owlvit-base-patch32": {"image_size": 768, "patch_size": 32, "processor": OwlViTProcessor, "max_text_length": 16},
    "google/owlvit-base-patch16": {"image_size": 768, "patch_size": 16, "processor": OwlViTProcessor, "max_text_length": 16},
    "google/owlvit-large-patch14": {"image_size": 840, "patch_size": 14, "processor": OwlViTProcessor, "max_text_length": 16},
}

def load_coco_captions(path, num_samples=4096):
    if not os.path.exists(path): raise FileNotFoundError(f"COCO captions file not found at: {path}")
    print(f"Loading captions from {path}...")
    with open(path, 'r') as f: data = json.load(f)
    all_captions = [item['caption'] for item in data['annotations']]
    random.shuffle(all_captions)
    num_to_sample = min(num_samples, len(all_captions))
    print(f"Sampling {num_to_sample} captions for calibration.")
    return all_captions[:num_to_sample]

def letterbox(im: Image.Image, new_shape: Tuple[int, int], color=(0, 0, 0)) -> np.ndarray:
    w, h = im.size
    target_h, target_w = new_shape
    r = min(target_h / h, target_w / w)
    new_w, new_h = int(w * r), int(h * r)
    
    resized_im = im.resize((new_w, new_h), Image.Resampling.BICUBIC)
    
    padded_im = Image.new("RGB", (target_w, target_h), color)
    padded_im.paste(resized_im, ((target_w - new_w) // 2, (target_h - new_h) // 2))
    
    return np.array(padded_im)

class ImageCalibrator(trt.IInt8EntropyCalibrator2):
    def __init__(self, input_shape, batch_size, data_dir, cache_file):
        trt.IInt8EntropyCalibrator2.__init__(self)
        self.input_shape, self.batch_size, self.cache_file, self.index = input_shape, batch_size, cache_file, 0
        image_pattern = os.path.join(data_dir, 'images', 'train2017', '*.jpg')
        self.image_files = glob.glob(image_pattern)
        if not self.image_files: raise FileNotFoundError(f"No images found in '{os.path.dirname(image_pattern)}'")
        np.random.shuffle(self.image_files)
        print(f"Found {len(self.image_files)} images for calibration.")
        buffer_size = int(self.batch_size * np.prod(self.input_shape) * np.dtype(np.float32).itemsize)
        self.device_input = cuda.mem_alloc(buffer_size)
        
    def get_batch_size(self): return self.batch_size

    def get_batch(self, names):
        if self.index >= len(self.image_files): return None
        end_idx = min(self.index + self.batch_size, len(self.image_files))
        current_batch_size = end_idx - self.index
        host_batch = np.empty((current_batch_size, *self.input_shape), dtype=np.float32)
        
        for i, file_path in enumerate(self.image_files[self.index:end_idx]):
            img = Image.open(file_path).convert('RGB')
            c, h, w = self.input_shape
            
            img_padded = letterbox(img, (h, w))
            
            img_np = (img_padded.astype(np.float32).transpose((2, 0, 1)) - NORMALIZATION_MEAN[:, None, None]) / NORMALIZATION_STD[:, None, None]
            host_batch[i] = img_np
            
        cuda.memcpy_htod(self.device_input, host_batch.ravel())
        self.index += current_batch_size
        return [int(self.device_input)]

    def read_calibration_cache(self):
        if os.path.exists(self.cache_file):
            with open(self.cache_file, "rb") as f: return f.read()
            
    def write_calibration_cache(self, cache):
        with open(self.cache_file, "wb") as f: f.write(cache)
        
    def free(self): self.device_input.free()

class TextCalibrator(trt.IInt8EntropyCalibrator2):
    def __init__(self, model_name, batch_size, max_seq_len, cache_file, captions_json_path):
        trt.IInt8EntropyCalibrator2.__init__(self)
        self.batch_size, self.max_seq_len, self.cache_file, self.index = batch_size, max_seq_len, cache_file, 0
        self.queries = load_coco_captions(captions_json_path, num_samples=5000)
        full_model_name = f"google/{model_name}"
        processor_class = MODEL_PROPERTIES[full_model_name]["processor"]
        self.processor = processor_class.from_pretrained(full_model_name)
        ids_buffer_size = int(self.batch_size * self.max_seq_len * np.dtype(np.int64).itemsize)
        mask_buffer_size = int(self.batch_size * self.max_seq_len * np.dtype(np.int64).itemsize)
        self.device_input_ids = cuda.mem_alloc(ids_buffer_size)
        self.device_attention_mask = cuda.mem_alloc(mask_buffer_size)
        self.bindings = [int(self.device_input_ids), int(self.device_attention_mask)]
    def get_batch_size(self): return self.batch_size
    def get_batch(self, names):
        if self.index >= len(self.queries): return None
        end_idx = min(self.index + self.batch_size, len(self.queries))
        current_batch = self.queries[self.index:end_idx]
        inputs = self.processor(text=current_batch, return_tensors="pt", padding="max_length", truncation=True, max_length=self.max_seq_len)
        input_ids = np.ascontiguousarray(inputs['input_ids'].numpy())
        attention_mask = np.ascontiguousarray(inputs['attention_mask'].numpy())
        cuda.memcpy_htod(self.device_input_ids, input_ids)
        cuda.memcpy_htod(self.device_attention_mask, attention_mask)
        self.index += len(current_batch)
        return self.bindings
    def read_calibration_cache(self):
        if os.path.exists(self.cache_file):
            with open(self.cache_file, "rb") as f: return f.read()
    def write_calibration_cache(self, cache):
        with open(self.cache_file, "wb") as f: f.write(cache)
    def free(self): self.device_input_ids.free(); self.device_attention_mask.free()

def generate_calibration_cache(args):
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    parser = trt.OnnxParser(network, logger)
    
    full_model_name = f"google/{args.model_name}"
    file_prefix = f"{args.file_tag}_" if args.file_tag else ""
    
    onnx_filename = f"{file_prefix}{args.component}.onnx"
    onnx_path = os.path.join("weights/nanoowl/onnx", onnx_filename)
    cache_path = onnx_path.replace(".onnx", ".cache")

    if not os.path.exists(onnx_path): raise FileNotFoundError(f"ONNX file not found: {onnx_path}")
    with open(onnx_path, "rb") as model:
        if not parser.parse(model.read()):
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            raise ValueError(f"Failed to parse ONNX file: {onnx_path}")
    print(f"Successfully parsed ONNX model: {onnx_path}")

    profile = builder.create_optimization_profile()
    props = MODEL_PROPERTIES[full_model_name]
    
    calibrator = None
    if args.component == 'image_encoder':
        h = w = props['image_size']
        shape = (3, h, w)
        profile.set_shape("image", min=(1, *shape), opt=(args.batch_size, *shape), max=(args.batch_size, *shape))
        calibrator = ImageCalibrator(shape, args.batch_size, args.data_dir, cache_path)
    elif args.component == 'text_encoder':
        max_len = props['max_text_length']
        profile.set_shape("input_ids", min=(1, 1), opt=(args.batch_size, max_len//2), max=(args.batch_size, max_len))
        profile.set_shape("attention_mask", min=(1, 1), opt=(args.batch_size, max_len//2), max=(args.batch_size, max_len))
        calibrator = TextCalibrator(args.model_name, args.batch_size, max_len, cache_path, args.captions_path)

    config.add_optimization_profile(profile)
    config.set_flag(trt.BuilderFlag.INT8)
    if calibrator is None: raise ValueError(f"Invalid component: {args.component}")
    
    config.int8_calibrator = calibrator
    print(f"\nStarting INT8 calibration for '{args.component}' component...")
    serialized_engine = builder.build_serialized_network(network, config)
    
    if serialized_engine is None: print("\nEngine building failed during calibration.")
    else: print(f"\nCalibration complete. Cache file for '{args.component}' saved to '{cache_path}'")
    
    calibrator.free()

if __name__ == "__main__":
    short_model_names = [name.replace("google/", "") for name in MODEL_PROPERTIES.keys()]
    parser = argparse.ArgumentParser(description="Generate INT8 calibration cache for Owl model components.")
    parser.add_argument("--component", required=True, type=str, choices=['image_encoder', 'text_encoder'])
    parser.add_argument("--model_name", required=True, type=str, choices=short_model_names)
    parser.add_argument("--file_tag", type=str, default="")
    parser.add_argument("--data_dir", type=str, default="datasets/coco128")
    parser.add_argument("--captions_path", type=str, default="datasets/annotations/captions_train2017.json")
    parser.add_argument("--batch_size", type=int, default=8)
    args = parser.parse_args()
    
    try:
        generate_calibration_cache(args)
    except Exception as e:
        import traceback
        print(f"\nAn unexpected error occurred: {e}")
        traceback.print_exc()