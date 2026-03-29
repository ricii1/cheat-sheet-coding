# vLLM Cheat Sheet

## Instalasi vLLM

### Instalasi dengan pip
```bash
# Instalasi dasar
pip install vllm

# Instalasi dengan CUDA support (NVIDIA GPU)
pip install vllm

# Instalasi dengan ROCm support (AMD GPU)
pip install vllm --extra-index-url https://download.pytorch.org/whl/rocm6.0
```
Penjelasan: vLLM otomatis mendeteksi GPU dan menginstall dependencies yang sesuai.

### Instalasi dengan Constraints (Recommended)
```bash
# Gunakan constraints untuk kompatibilitas terbaik
pip install vllm --constraint constraints.txt

# Download constraints dari official repo
wget https://raw.githubusercontent.com/vllm-project/vllm/main/requirements-cuda.txt
```
Penjelasan: Constraints file memastikan kompatibilitas dependencies.

### Instalasi dari Source
```bash
# Clone repository
git clone https://github.com/vllm-project/vllm.git
cd vllm

# Install dari source
pip install -e .

# Install untuk development
pip install -e . -v
```
Penjelasan: Install dari source untuk akses ke fitur terbaru atau development.

### Instalasi dengan Docker
```bash
# Pull Docker image
docker pull vllm/vllm-openai:latest

# Jalankan container dengan GPU
docker run --runtime nvidia --gpus all \
    -p 8000:8000 \
    -v ~/.cache/huggingface:/root/.cache/huggingface \
    vllm/vllm-openai:latest \
    --model llama3.2
```
Penjelasan: Docker memberikan environment yang konsisten dan isolasi dependencies.

### Cek Versi vLLM
```bash
python -c "import vllm; print(vllm.__version__)"
```

### Update vLLM
```bash
# Update ke versi terbaru
pip install --upgrade vllm

# Update dari source
cd vllm
git pull
pip install -e .
```
Penjelasan: Update vLLM untuk mendapatkan performance improvements dan bug fixes.

## Menjalankan vLLM Server

### OpenAI-Compatible API Server
```bash
# Jalankan server dengan model tertentu
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct

# Jalankan dengan port custom
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --port 8080

# Jalankan dengan host custom
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --host 0.0.0.0 \
    --port 8000
```
Penjelasan: vLLM menyediakan API yang kompatibel dengan OpenAI API, memudahkan migrasi.

### Server dengan Model Lokal
```bash
# Gunakan model dari Hugging Face
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-3B-Instruct

# Gunakan model dari path lokal
python -m vllm.entrypoints.openai.api_server \
    --model /path/to/local/model

# Gunakan model dari Ollama format
python -m vllm.entrypoints.openai.api_server \
    --model ollama://llama3.2
```
Penjelasan: vLLM support berbagai sumber model termasuk Hugging Face, lokal, dan Ollama.

### Server dengan Quantization
```bash
# AWQ quantization
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --quantization awq

# GPTQ quantization
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --quantization gptq

# SqueezeLLM quantization
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --quantization squeezellm
```
Penjelasan: Quantization mengurangi memory usage dan meningkatkan throughput.

### Server dengan Multi-GPU
```bash
# Tensor parallelism (split model layers across GPUs)
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --tensor-parallel-size 2

# Pipeline parallelism (split model stages across GPUs)
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --pipeline-parallel-size 2

# Combined parallelism
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --tensor-parallel-size 2 \
    --pipeline-parallel-size 2
```
Penjelasan: Parallelism memungkinkan running model besar di multiple GPUs.

### Server dengan Custom Config
```bash
# Set context length
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --max-model-len 8192

# Set batch size
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --max-num-batched-tokens 10000

# Set GPU memory utilization
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --gpu-memory-utilization 0.9
```
Penjelasan: Custom config untuk optimasi berdasarkan hardware yang tersedia.

### Background Service (systemd)
```ini
# /etc/systemd/system/vllm.service
[Unit]
Description=vLLM OpenAI-Compatible Server
After=network.target

[Service]
User=youruser
Group=youruser
WorkingDirectory=/path/to/working/dir
Environment="HUGGING_FACE_HUB_TOKEN=your_token"
ExecStart=/usr/bin/python3 -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --host 0.0.0.0 \
    --port 8000
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```
Penjelasan: Systemd service untuk menjalankan vLLM sebagai background service.

```bash
# Enable dan start service
sudo systemctl daemon-reload
sudo systemctl enable vllm
sudo systemctl start vllm

# Cek status
sudo systemctl status vllm

# Lihat logs
journalctl -u vllm -f
```

## Menggunakan vLLM API

### Chat Completions API
```bash
# Simple chat request
curl http://localhost:8000/v1/chat/completions \
    -H "Content-Type: application/json" \
    -d '{
        "model": "meta-llama/Llama-3.2-1B-Instruct",
        "messages": [
            {"role": "user", "content": "Halo, apa kabar?"}
        ]
    }'
```
Penjelasan: Endpoint chat kompatibel dengan OpenAI API format.

### Chat dengan Conversation History
```bash
curl http://localhost:8000/v1/chat/completions \
    -H "Content-Type: application/json" \
    -d '{
        "model": "meta-llama/Llama-3.2-1B-Instruct",
        "messages": [
            {"role": "system", "content": "Kamu adalah asisten yang membantu."},
            {"role": "user", "content": "Halo!"},
            {"role": "assistant", "content": "Halo! Ada yang bisa saya bantu?"},
            {"role": "user", "content": "Apa kabar?"}
        ]
    }'
```
Penjelasan: Conversation history memberikan konteks untuk response yang lebih baik.

### Chat dengan Parameters
```bash
curl http://localhost:8000/v1/chat/completions \
    -H "Content-Type: application/json" \
    -d '{
        "model": "meta-llama/Llama-3.2-1B-Instruct",
        "messages": [{"role": "user", "content": "Jelaskan AI"}],
        "temperature": 0.7,
        "top_p": 0.9,
        "max_tokens": 512,
        "stream": true
    }'
```
Penjelasan: Parameters mengontrol behavior generation seperti temperature dan max tokens.

### Streaming Response
```bash
curl http://localhost:8000/v1/chat/completions \
    -H "Content-Type: application/json" \
    -d '{
        "model": "meta-llama/Llama-3.2-1B-Instruct",
        "messages": [{"role": "user", "content": "Jelaskan machine learning"}],
        "stream": true
    }'
```
Penjelasan: Streaming mengembalikan response chunk by chunk untuk real-time display.

### Completions API (Legacy)
```bash
curl http://localhost:8000/v1/completions \
    -H "Content-Type: application/json" \
    -d '{
        "model": "meta-llama/Llama-3.2-1B-Instruct",
        "prompt": "Once upon a time",
        "max_tokens": 100
    }'
```
Penjelasan: Legacy completions API untuk text generation tanpa chat format.

### List Models API
```bash
curl http://localhost:8000/v1/models
```
Penjelasan: Mendapatkan daftar model yang tersedia di server.

### Embeddings API
```bash
curl http://localhost:8000/v1/embeddings \
    -H "Content-Type: application/json" \
    -d '{
        "model": "meta-llama/Llama-3.2-1B-Instruct",
        "input": "This is a sample text for embedding"
    }'
```
Penjelasan: Generate embeddings untuk semantic search atau RAG applications.

## Python Client

### Menggunakan OpenAI SDK
```python
from openai import OpenAI

# Initialize client
client = OpenAI(
    base_url="http://localhost:8000/v1",
    api_key="not-needed"  # vLLM tidak require API key
)

# Chat completion
response = client.chat.completions.create(
    model="meta-llama/Llama-3.2-1B-Instruct",
    messages=[
        {"role": "user", "content": "Halo, apa kabar?"}
    ]
)

print(response.choices[0].message.content)
```
Penjelasan: OpenAI SDK bisa langsung digunakan dengan vLLM karena API kompatibel.

### Streaming dengan Python
```python
from openai import OpenAI

client = OpenAI(base_url="http://localhost:8000/v1", api_key="not-needed")

stream = client.chat.completions.create(
    model="meta-llama/Llama-3.2-1B-Instruct",
    messages=[{"role": "user", "content": "Jelaskan AI"}],
    stream=True
)

for chunk in stream:
    if chunk.choices[0].delta.content:
        print(chunk.choices[0].delta.content, end="")
```
Penjelasan: Streaming response untuk real-time text generation.

### Async Python Client
```python
import asyncio
from openai import AsyncOpenAI

client = AsyncOpenAI(base_url="http://localhost:8000/v1", api_key="not-needed")

async def main():
    response = await client.chat.completions.create(
        model="meta-llama/Llama-3.2-1B-Instruct",
        messages=[{"role": "user", "content": "Halo!"}]
    )
    print(response.choices[0].message.content)

asyncio.run(main())
```
Penjelasan: Async client untuk concurrent requests dan better performance.

### Batch Requests
```python
from openai import OpenAI

client = OpenAI(base_url="http://localhost:8000/v1", api_key="not-needed")

responses = client.chat.completions.create(
    model="meta-llama/Llama-3.2-1B-Instruct",
    messages=[
        {"role": "user", "content": "Apa itu Python?"},
        {"role": "user", "content": "Apa itu JavaScript?"},
        {"role": "user", "content": "Apa itu Go?"}
    ]
)
```
Penjelasan: Batch requests untuk efisiensi ketika ada multiple queries.

## Advanced Configuration

### Memory Management
```bash
# Set GPU memory utilization ratio
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --gpu-memory-utilization 0.95

# Swap space untuk CPU offloading
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --swap-space-size 4

# Max model length
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --max-model-len 16384
```
Penjelasan: Memory management untuk optimasi GPU utilization.

### Scheduler Configuration
```bash
# Scheduling policy
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --scheduler-delay-factor 0.0

# Max number of sequences
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --max-num-seqs 256
```
Penjelasan: Scheduler configuration untuk mengontrol request handling.

### Logging Configuration
```bash
# Set log level
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --log-level INFO

# Enable detailed logging
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --log-requests
```
Penjelasan: Logging untuk monitoring dan debugging.

### LoRA Adapters
```bash
# Enable LoRA
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --enable-lora \
    --lora-modules name1=/path/to/lora1,name2=/path/to/lora2

# Max LoRA rank
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --enable-lora \
    --max-lora-rank 64
```
Penjelasan: LoRA adapters untuk fine-tuned models tanpa full model reload.

### Speculative Decoding
```bash
# Enable speculative decoding dengan draft model
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-3B-Instruct \
    --speculative-model meta-llama/Llama-3.2-1B-Instruct \
    --num-speculative-tokens 5
```
Penjelasan: Speculative decoding meningkatkan throughput dengan draft model.

## Performance Tuning

### Throughput Optimization
```bash
# Increase batch size
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --max-num-batched-tokens 20000 \
    --max-num-seqs 512

# Enable chunked prefill
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --enable-chunked-prefill
```
Penjelasan: Throughput optimization untuk high-volume workloads.

### Latency Optimization
```bash
# Reduce batch size untuk lower latency
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --max-num-seqs 64 \
    --scheduler-delay-factor 0.1
```
Penjelasan: Latency optimization untuk real-time applications.

### Benchmark Performance
```bash
# Gunakan benchmark tool
python benchmarks/benchmark_throughput.py \
    --backend vllm \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --input-len 1024 \
    --output-len 256
```
Penjelasan: Benchmark untuk mengukur performance dan tuning parameters.

## Multi-Model Serving

### Serving Multiple Models
```bash
# vLLM support serving multiple models secara bergantian
# Gunakan different ports untuk different models

# Terminal 1
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --port 8001

# Terminal 2
python -m vllm.entrypoints.openai.api_server \
    --model codellama/CodeLlama-7b-Instruct-hf \
    --port 8002
```
Penjelasan: Multiple instances untuk serving different models.

## Troubleshooting

### Out of Memory (OOM)
```bash
# Reduce GPU memory utilization
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --gpu-memory-utilization 0.8

# Use smaller model
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct

# Enable quantization
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3.2-1B-Instruct \
    --quantization awq
```
Penjelasan: OOM terjadi ketika model terlalu besar untuk available GPU memory.

### Model Loading Issues
```bash
# Check Hugging Face token
export HUGGING_FACE_HUB_TOKEN=your_token

# Use local model cache
python -m vllm.entrypoints.openai.api_server \
    --model /path/to/cached/model

# Clear cache dan re-download
rm -rf ~/.cache/huggingface
```
Penjelasan: Model loading issues sering karena authentication atau corrupt cache.

### Performance Issues
```bash
# Check GPU utilization
nvidia-smi

# Check vLLM logs
journalctl -u vllm -f

# Monitor request latency
curl http://localhost:8000/metrics
```
Penjelasan: Performance monitoring untuk identify bottlenecks.

### CUDA Compatibility Issues
```bash
# Check CUDA version
nvcc --version

# Check PyTorch CUDA
python -c "import torch; print(torch.cuda.is_available())"

# Reinstall with correct CUDA version
pip uninstall vllm
pip install vllm
```
Penjelasan: CUDA compatibility penting untuk GPU acceleration.

## Command Reference Cepat

```bash
# Installation
pip install vllm                    # Install vLLM
pip install --upgrade vllm          # Upgrade vLLM

# Server
python -m vllm.entrypoints.openai.api_server --model <model>  # Start server
python -m vllm.entrypoints.openai.api_server --model <model> --port 8080  # Custom port

# Configuration
--tensor-parallel-size N            # Number of GPUs for tensor parallelism
--pipeline-parallel-size N          # Number of GPUs for pipeline parallelism
--gpu-memory-utilization 0.9        # GPU memory ratio
--max-model-len 8192                # Maximum context length
--quantization awq                  # Quantization method
--enable-lora                       # Enable LoRA adapters

# API Endpoints
/v1/chat/completions               # Chat completions
/v1/completions                    # Text completions
/v1/models                         # List models
/v1/embeddings                     # Generate embeddings
/metrics                           # Prometheus metrics
```
Penjelasan: Daftar command dan API endpoint yang paling sering digunakan.

## Tips dan Best Practices

### Pilih Model yang Tepat
- **Llama-3.2-1B/3B**: Lightweight, fast inference
- **Llama-3.2-7B/70B**: General purpose, good balance
- **CodeLlama**: Optimized untuk coding tasks
- **Mistral**: Efficient, good performance
- **Qwen**: Multilingual support

### Production Deployment
```bash
# Gunakan reverse proxy (nginx)
# Enable HTTPS
# Set rate limiting
# Monitor dengan Prometheus/Grafana
# Enable logging dan alerting
```
Penjelasan: Production deployment memerlukan additional considerations untuk security dan reliability.

### Environment Variables
```bash
export HUGGING_FACE_HUB_TOKEN=your_token
export VLLM_NO_USAGE_STATS=1
export VLLM_ALLOW_LONG_MAX_MODEL_LEN=1
```
Penjelasan: Environment variables untuk configuration dan authentication.

### Monitoring dengan Prometheus
```bash
# vLLM expose metrics di /metrics endpoint
curl http://localhost:8000/metrics

# Metrics yang tersedia:
# - vllm:num_requests_running
# - vllm:num_requests_waiting
# - vllm:avg_prompt_tokenization_latency
# - vllm:avg_generation_throughput
```
Penjelasan: Metrics untuk monitoring performance dan health.

### Integration dengan Popular Frameworks

**LangChain:**
```python
from langchain_community.llms import VLLMOpenAI

llm = VLLMOpenAI(
    openai_api_key="not-needed",
    openai_api_base="http://localhost:8000/v1",
    model_name="meta-llama/Llama-3.2-1B-Instruct"
)
```

**LlamaIndex:**
```python
from llama_index.llms.openai_like import OpenAILike

llm = OpenAILike(
    api_key="not-needed",
    api_base="http://localhost:8000/v1",
    model="meta-llama/Llama-3.2-1B-Instruct"
)
```
Penjelasan: vLLM terintegrasi dengan popular LLM frameworks.
