# Ollama Cheat Sheet

- [Ollama Cheat Sheet](#ollama-cheat-sheet)
  - [Instalasi Ollama](#instalasi-ollama)
    - [Instalasi di Linux](#instalasi-di-linux)
    - [Instalasi di macOS](#instalasi-di-macos)
    - [Instalasi di Windows](#instalasi-di-windows)
    - [Cek Versi Ollama](#cek-versi-ollama)
    - [Update Ollama](#update-ollama)
    - [Memulai Ollama Server](#memulai-ollama-server)
    - [Jalankan Server di Background](#jalankan-server-di-background)
  - [Mengelola Model](#mengelola-model)
    - [Download/Pull Model](#downloadpull-model)
    - [Melihat Daftar Model yang Sudah Di-download](#melihat-daftar-model-yang-sudah-di-download)
    - [Menjalankan Model (Chat Mode)](#menjalankan-model-chat-mode)
    - [Menghapus Model](#menghapus-model)
    - [Copy Model](#copy-model)
    - [Pull Model dari Registry Lain](#pull-model-dari-registry-lain)
  - [Running Model dengan Options](#running-model-dengan-options)
    - [Set Context Window Size](#set-context-window-size)
    - [Set Temperature](#set-temperature)
    - [Set Top P](#set-top-p)
    - [Set Number of Predictions](#set-number-of-predictions)
    - [Set Repeat Penalty](#set-repeat-penalty)
  - [Ollama API](#ollama-api)
    - [Generate Response (API)](#generate-response-api)
    - [Chat Completions (API)](#chat-completions-api)
    - [Generate dengan Stream](#generate-dengan-stream)
    - [List Models (API)](#list-models-api)
    - [Show Model Info (API)](#show-model-info-api)
    - [Delete Model (API)](#delete-model-api)
    - [Pull Model (API)](#pull-model-api)
    - [Copy Model (API)](#copy-model-api)
  - [Membuat Custom Model (Modelfile)](#membuat-custom-model-modelfile)
    - [Membuat Modelfile Sederhana](#membuat-modelfile-sederhana)
    - [Membuat Custom Model dari Modelfile](#membuat-custom-model-dari-modelfile)
    - [Modelfile dengan Template Custom](#modelfile-dengan-template-custom)
    - [Modelfile dengan Adapter (LoRA)](#modelfile-dengan-adapter-lora)
    - [Melihat Isi Modelfile Model](#melihat-isi-modelfile-model)
  - [Embeddings](#embeddings)
    - [Generate Embeddings](#generate-embeddings)
    - [Embeddings dengan Model Khusus](#embeddings-dengan-model-khusus)
  - [Multi-Modal Models](#multi-modal-models)
    - [Model Vision (Gambar ke Text)](#model-vision-gambar-ke-text)
    - [Vision API](#vision-api)
  - [Configuration dan Environment Variables](#configuration-dan-environment-variables)
    - [Environment Variables](#environment-variables)
    - [Konfigurasi Persistent](#konfigurasi-persistent)
    - [Set Keep Alive Time](#set-keep-alive-time)
  - [Troubleshooting](#troubleshooting)
    - [Cek Status Server](#cek-status-server)
    - [Model Tidak Bisa Di-load](#model-tidak-bisa-di-load)
    - [Out of Memory](#out-of-memory)
    - [Reset Ollama](#reset-ollama)
  - [Command Reference Cepat](#command-reference-cepat)
  - [Tips dan Best Practices](#tips-dan-best-practices)
    - [Pilih Model yang Tepat](#pilih-model-yang-tepat)
    - [Optimasi Performance](#optimasi-performance)
    - [Chat Commands (Dalam Interactive Mode)](#chat-commands-dalam-interactive-mode)
    - [Integrasi dengan Aplikasi Lain](#integrasi-dengan-aplikasi-lain)

## Instalasi Ollama

### Instalasi di Linux
```bash
# Download dan jalankan installer
curl -fsSL https://ollama.com/install.sh | sh
```
Penjelasan: Script installer akan mendeteksi OS dan menginstall Ollama dengan dependencies yang diperlukan.

### Instalasi di macOS
```bash
# Download dari website atau gunakan Homebrew
brew install ollama
```
Penjelasan: Homebrew akan menginstall Ollama beserta dependencies-nya secara otomatis.

### Instalasi di Windows
```bash
# Download installer dari https://ollama.com/download/windows
# Jalankan installer dan ikuti instruksi
```
Penjelasan: Windows installer akan setup Ollama sebagai aplikasi desktop.

### Cek Versi Ollama
```bash
ollama --version
```

### Update Ollama
```bash
# Linux/macOS - jalankan installer lagi
curl -fsSL https://ollama.com/install.sh | sh

# macOS dengan Homebrew
brew upgrade ollama
```
Penjelasan: Update Ollama untuk mendapatkan model dan fitur terbaru.

### Memulai Ollama Server
```bash
# Jalankan server (default port 11434)
ollama serve
```
Penjelasan: Server harus berjalan untuk bisa menggunakan model. Default bind ke 127.0.0.1:11434.

### Jalankan Server di Background
```bash
# Linux dengan systemd
sudo systemctl start ollama
sudo systemctl enable ollama

# Atau gunakan nohup
nohup ollama serve &
```
Penjelasan: Server berjalan di background agar tetap aktif setelah terminal ditutup.

## Mengelola Model

### Download/Pull Model
```bash
# Download model terbaru
ollama pull llama3.2

# Download model dengan tag tertentu
ollama pull llama3.2:7b

# Download model lain
ollama pull mistral
ollama pull codellama
ollama pull gemma2
ollama pull qwen2.5
```
Penjelasan: Model akan di-download dari Ollama library dan disimpan lokal untuk digunakan offline.

### Melihat Daftar Model yang Sudah Di-download
```bash
ollama list
```
atau
```bash
ollama ls
```
Penjelasan: Menampilkan semua model yang tersedia beserta ukuran dan waktu modifikasi.

### Menjalankan Model (Chat Mode)
```bash
# Jalankan model untuk chat interaktif
ollama run llama3.2

# Jalankan model dengan prompt langsung
ollama run llama3.2 "Jelaskan apa itu machine learning"

# Jalankan model dengan multi-line prompt
ollama run llama3.2 << "EOF"
Jelaskan perbedaan supervised dan unsupervised learning
Berikan contoh masing-masing
EOF
```
Penjelasan: Chat mode memungkinkan interaksi conversational dengan model AI.

### Menghapus Model
```bash
# Hapus model tertentu
ollama rm llama3.2

# Hapus model dengan tag tertentu
ollama rm llama3.2:7b
```
Penjelasan: Menghapus model dari storage lokal untuk menghemat space.

### Copy Model
```bash
# Copy model dengan nama baru
ollama cp llama3.2 my-llama
```
Penjelasan: Berguna untuk membuat varian model dengan konfigurasi berbeda.

### Pull Model dari Registry Lain
```bash
# Pull dari registry custom
OLLAMA_HOST=https://registry.example.com ollama pull model-name
```
Penjelasan: Ollama support multiple registries untuk model distribution.

## Running Model dengan Options

### Set Context Window Size
```bash
# Jalankan dengan context length tertentu
ollama run llama3.2 --num_ctx 4096
```
Penjelasan: Context length menentukan berapa banyak token yang bisa diproses dalam satu sesi.

### Set Temperature
```bash
# Temperature lebih tinggi = lebih kreatif/random
ollama run llama3.2 --temperature 0.8
```
Penjelasan: Temperature mengontrol randomness output. Rendah = deterministik, Tinggi = kreatif.

### Set Top P
```bash
# Nucleus sampling
ollama run llama3.2 --top_p 0.9
```
Penjelasan: Top P membatasi sampling ke token dengan cumulative probability tertentu.

### Set Number of Predictions
```bash
# Generate multiple predictions
ollama run llama3.2 --num_predict 512
```
Penjelasan: Membatasi jumlah maksimum token yang akan digenerate.

### Set Repeat Penalty
```bash
# Hindari pengulangan
ollama run llama3.2 --repeat_penalty 1.2
```
Penjelasan: Menghukum token yang sudah muncul sebelumnya untuk mengurangi repetisi.

## Ollama API

### Generate Response (API)
```bash
# Simple generate request
curl http://localhost:11434/api/generate -d '{
  "model": "llama3.2",
  "prompt": "Mengapa langit berwarna biru?"
}'
```
Penjelasan: API endpoint untuk generate response tanpa mode chat.

### Chat Completions (API)
```bash
# Chat dengan conversation history
curl http://localhost:11434/api/chat -d '{
  "model": "llama3.2",
  "messages": [
    {"role": "user", "content": "Halo!"},
    {"role": "assistant", "content": "Halo! Ada yang bisa saya bantu?"},
    {"role": "user", "content": "Apa kabar?"}
  ]
}'
```
Penjelasan: Endpoint chat yang mendukung conversation history untuk konteks percakapan.

### Generate dengan Stream
```bash
# Streaming response
curl http://localhost:11434/api/generate -d '{
  "model": "llama3.2",
  "prompt": "Jelaskan AI",
  "stream": true
}'
```
Penjelasan: Stream mode mengembalikan response secara real-time saat digenerate.

### List Models (API)
```bash
curl http://localhost:11434/api/tags
```
Penjelasan: Mendapatkan daftar model yang tersedia via API.

### Show Model Info (API)
```bash
curl http://localhost:11434/api/show -d '{
  "name": "llama3.2"
}'
```
Penjelasan: Mendapatkan detail informasi tentang model tertentu.

### Delete Model (API)
```bash
curl -X DELETE http://localhost:11434/api/delete -d '{
  "name": "llama3.2"
}'
```
Penjelasan: Menghapus model via API endpoint.

### Pull Model (API)
```bash
curl http://localhost:11434/api/pull -d '{
  "name": "llama3.2"
}'
```
Penjelasan: Download model via API endpoint.

### Copy Model (API)
```bash
curl http://localhost:11434/api/copy -d '{
  "source": "llama3.2",
  "destination": "my-llama"
}'
```
Penjelasan: Copy model via API endpoint.

## Membuat Custom Model (Modelfile)

### Membuat Modelfile Sederhana
```bash
# Buat file Modelfile
cat > Modelfile << EOF
FROM llama3.2

SYSTEM """
Kamu adalah asisten AI yang membantu dan ramah.
Jawab dengan bahasa Indonesia yang baik.
"""

PARAMETER temperature 0.7
PARAMETER top_p 0.9
EOF
```
Penjelasan: Modelfile mendefinisikan konfigurasi custom untuk model.

### Membuat Custom Model dari Modelfile
```bash
# Build model dari Modelfile
ollama create my-assistant -f Modelfile
```
Penjelasan: Compile Modelfile menjadi model yang bisa digunakan.

### Modelfile dengan Template Custom
```bash
cat > Modelfile << EOF
FROM llama3.2

TEMPLATE """
<|user|>
{{ .Prompt }}
<|assistant|>
"""

SYSTEM "Kamu adalah coding assistant yang ahli dalam Python dan JavaScript."

PARAMETER temperature 0.5
PARAMETER num_ctx 2048
EOF

ollama create coding-assistant -f Modelfile
```
Penjelasan: Template custom mengontrol format input ke model.

### Modelfile dengan Adapter (LoRA)
```bash
cat > Modelfile << EOF
FROM llama3.2
ADAPTER ./path/to/lora-adapter
EOF

ollama create finetuned-model -f Modelfile
```
Penjelasan: Adapter LoRA untuk fine-tuning model tanpa mengubah weights utama.

### Melihat Isi Modelfile Model
```bash
ollama show llama3.2 --modelfile
```
Penjelasan: Melihat konfigurasi Modelfile dari model yang sudah ada.

## Embeddings

### Generate Embeddings
```bash
# Generate embedding untuk text
curl http://localhost:11434/api/embeddings -d '{
  "model": "llama3.2",
  "prompt": "Ini adalah contoh text untuk embedding"
}'
```
Penjelasan: Embeddings berguna untuk semantic search, RAG, dan similarity comparison.

### Embeddings dengan Model Khusus
```bash
# Gunakan model yang optimized untuk embeddings
ollama pull nomic-embed-text
curl http://localhost:11434/api/embeddings -d '{
  "model": "nomic-embed-text",
  "prompt": "Text untuk embedding"
}'
```
Penjelasan: Model embeddings khusus biasanya lebih efisien untuk tugas retrieval.

## Multi-Modal Models

### Model Vision (Gambar ke Text)
```bash
# Pull model vision
ollama pull llava

# Chat dengan gambar
ollama run llava "Apa yang ada di gambar ini?" /path/to/image.jpg
```
Penjelasan: Model vision bisa menganalisis dan mendeskripsikan konten gambar.

### Vision API
```bash
# API call dengan gambar (base64 encoded)
curl http://localhost:11434/api/generate -d '{
  "model": "llava",
  "prompt": "Deskripsikan gambar ini",
  "images": ["/path/to/image.jpg"]
}'
```
Penjelasan: Vision API memungkinkan integrasi gambar analysis ke aplikasi.

## Configuration dan Environment Variables

### Environment Variables
```bash
# Set custom port
OLLAMA_HOST=0.0.0.0:11435 ollama serve

# Set origin untuk CORS
OLLAMA_ORIGINS="https://example.com,https://app.example.com" ollama serve

# Set custom models directory
OLLAMA_MODELS=/path/to/models ollama serve
```
Penjelasan: Environment variables mengontrol perilaku server Ollama.

### Konfigurasi Persistent
```bash
# Linux - edit systemd service
sudo systemctl edit ollama

# Tambahkan environment variables
[Service]
Environment="OLLAMA_HOST=0.0.0.0:11434"
Environment="OLLAMA_ORIGINS=*"
```
Penjelasan: Konfigurasi persistent agar setting tetap ada setelah restart.

### Set Keep Alive Time
```bash
# Model tetap loaded untuk waktu tertentu (dalam detik)
ollama run llama3.2
# Dalam chat: /set parameter keep_alive 300
```
Penjelasan: Keep alive mencegah model unload dari memory terlalu cepat.

## Troubleshooting

### Cek Status Server
```bash
# Cek apakah server berjalan
curl http://localhost:11434

# Lihat logs
journalctl -u ollama -f

# Atau jika run manual, lihat output console
```
Penjelasan: Troubleshooting dasar untuk memastikan server berjalan dengan benar.

### Model Tidak Bisa Di-load
```bash
# Cek space disk
df -h

# Cek model yang corrupt
ollama list

# Re-download model
ollama rm model-name
ollama pull model-name
```
Penjelasan: Masalah umum adalah disk penuh atau model file corrupt.

### Out of Memory
```bash
# Gunakan model yang lebih kecil
ollama pull llama3.2:1b

# Kurangi context length
ollama run llama3.2 --num_ctx 1024

# Set GPU layers (untuk NVIDIA)
OLLAMA_NUM_GPU=20 ollama serve
```
Penjelasan: OOM terjadi ketika model terlalu besar untuk available memory.

### Reset Ollama
```bash
# Stop server
sudo systemctl stop ollama

# Hapus semua models
rm -rf ~/.ollama/models

# Restart server
sudo systemctl start ollama
```
Penjelasan: Reset penuh untuk troubleshooting masalah yang persisten.

## Command Reference Cepat

```bash
# Server Management
ollama serve                    # Jalankan server
ollama --version               # Cek versi

# Model Management
ollama pull model              # Download model
ollama list                    # List model
ollama rm model                # Hapus model
ollama cp source dest          # Copy model
ollama show model              # Info model

# Running Models
ollama run model               # Chat mode
ollama run model "prompt"      # One-shot prompt
ollama create name -f file     # Buat custom model

# API Endpoints
/api/generate                  # Generate response
/api/chat                      # Chat completions
/api/embeddings               # Generate embeddings
/api/tags                     # List models
/api/pull                     # Pull model
/api/delete                   # Delete model
/api/show                     # Show model info
/api/copy                     # Copy model
```
Penjelasan: Daftar command dan API endpoint yang paling sering digunakan.

## Tips dan Best Practices

### Pilih Model yang Tepat
- **llama3.2**: General purpose, baik untuk berbagai tugas
- **codellama**: Optimized untuk coding tasks
- **mistral**: Ringan dan cepat, good untuk general use
- **gemma2**: Efficient, good performance/size ratio
- **qwen2.5**: Multilingual, baik untuk bahasa non-Inggris

### Optimasi Performance
```bash
# Gunakan GPU jika tersedia
nvidia-smi  # Cek GPU NVIDIA

# Set quantization untuk model lebih kecil
ollama pull llama3.2:q4_0  # 4-bit quantized

# Batch requests untuk throughput lebih tinggi
```
Penjelasan: Quantization mengurangi ukuran model dengan trade-off akurasi minimal.

### Chat Commands (Dalam Interactive Mode)
```bash
/set system <prompt>     # Set system prompt
/set temperature <val>   # Set temperature
/set top_p <val>         # Set top_p
/set num_ctx <val>       # Set context length
/show info               # Show model info
/show modelfile          # Show modelfile
/help                    # Show help
bye                      # Exit chat
```
Penjelasan: Commands yang tersedia saat dalam interactive chat mode.

### Integrasi dengan Aplikasi Lain
```bash
# Python dengan requests
import requests
response = requests.post('http://localhost:11434/api/generate', json={
    'model': 'llama3.2',
    'prompt': 'Hello!'
})

# Node.js dengan fetch
const response = await fetch('http://localhost:11434/api/generate', {
    method: 'POST',
    body: JSON.stringify({ model: 'llama3.2', prompt: 'Hello!' })
})
```
Penjelasan: Ollama API mudah diintegrasikan dengan berbagai programming language.
