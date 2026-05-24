#!/usr/bin/env python3
"""Pre-build a TensorRT FP16 engine from an ONNX model.

Run this OFFLINE (when no other GPU workload is active) to avoid OOM
during the expensive engine optimization phase.

Usage:
    python3 build_engine.py src/vision/vision/yolov8n.onnx
    python3 build_engine.py src/vision/vision/yolov8n.onnx --workspace_mb 128 --fp16
"""

import argparse
import gc
import os
import sys
import time


def build(onnx_path: str, workspace_mb: int, fp16: bool):
    import tensorrt as trt

    engine_path = onnx_path.replace('.onnx', '.engine')
    logger = trt.Logger(trt.Logger.INFO)

    print(f'ONNX model : {onnx_path} ({os.path.getsize(onnx_path) / 1024 / 1024:.1f} MB)')
    print(f'Engine out : {engine_path}')
    print(f'Workspace  : {workspace_mb} MB')
    print(f'FP16       : {fp16}')
    print()

    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, logger)

    with open(onnx_path, 'rb') as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(f'  Parse error: {parser.get_error(i)}')
            sys.exit(1)

    config = builder.create_builder_config()
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE, workspace_mb * (1 << 20)
    )

    if fp16 and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
        print('FP16 enabled on this platform')
    elif fp16:
        print('WARNING: FP16 requested but not supported — building FP32')

    print('Building engine (this takes 2-10 minutes)...')
    t0 = time.perf_counter()
    serialized = builder.build_serialized_network(network, config)
    elapsed = time.perf_counter() - t0

    if serialized is None:
        print('ERROR: Engine build failed (likely OOM). Try --workspace_mb 64')
        sys.exit(1)

    with open(engine_path, 'wb') as f:
        f.write(serialized)

    del builder, network, parser, config, serialized
    gc.collect()

    size_mb = os.path.getsize(engine_path) / 1024 / 1024
    print(f'Done in {elapsed:.1f}s — {engine_path} ({size_mb:.1f} MB)')
    print('The detector will now load this engine directly, skipping the build step.')


def main():
    parser = argparse.ArgumentParser(
        description='Pre-build TensorRT engine from ONNX (run offline)')
    parser.add_argument('onnx', help='Path to .onnx model')
    parser.add_argument('--workspace_mb', type=int, default=128,
                        help='TRT workspace size in MB (default: 128)')
    parser.add_argument('--fp16', action='store_true', default=True,
                        help='Enable FP16 (default: True)')
    parser.add_argument('--fp32', action='store_true',
                        help='Force FP32 (disable FP16)')
    args = parser.parse_args()

    if not os.path.exists(args.onnx):
        sys.exit(f'File not found: {args.onnx}')

    build(args.onnx, args.workspace_mb, fp16=not args.fp32)


if __name__ == '__main__':
    main()
