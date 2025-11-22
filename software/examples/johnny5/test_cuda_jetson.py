import torch
import time

def test_cuda():
    print("\n=== Jetson Orin CUDA Verification ===")
    
    print(f"PyTorch Version: {torch.__version__}")
    
    if torch.cuda.is_available():
        print("✅ CUDA is available!")
        device = torch.device("cuda")
        print(f"Device Name: {torch.cuda.get_device_name(0)}")
        print(f"Device Count: {torch.cuda.device_count()}")
        
        print("\nRunning Tensor Benchmark...")
        x = torch.rand(10000, 10000).to(device)
        y = torch.rand(10000, 10000).to(device)
        
        start = time.time()
        z = torch.matmul(x, y)
        end = time.time()
        
        print(f"Matrix Multiplication (10k x 10k) Time: {end - start:.4f} seconds")
    else:
        print("❌ CUDA is NOT available. Running on CPU.")

if __name__ == "__main__":
    test_cuda()