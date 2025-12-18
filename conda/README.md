# Conda Cheat Sheet

## Instalasi Conda

### Instalasi Miniconda (Recommended)
```bash
# Download installer
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# Jalankan installer
bash Miniconda3-latest-Linux-x86_64.sh

# Ikuti instruksi, ketik 'yes' untuk init conda
# Restart terminal setelah selesai
```
Penjelasan: Miniconda adalah versi ringan dari Anaconda yang hanya berisi conda dan Python.

### Instalasi Anaconda (Full Package)
```bash
# Download installer dari https://www.anaconda.com/download
wget https://repo.anaconda.com/archive/Anaconda3-latest-Linux-x86_64.sh

# Jalankan installer
bash Anaconda3-latest-Linux-x86_64.sh
```
Penjelasan: Anaconda sudah include banyak package data science, lebih besar dari Miniconda.

### Cek Versi Conda
```bash
conda --version
```

### Update Conda
```bash
conda update conda
```
Penjelasan: Update conda ke versi terbaru untuk bug fixes dan fitur baru.

## Mengelola Environment

### Membuat Environment Baru
```bash
# Buat environment dengan Python versi tertentu
conda create -n nama_env python=3.10

# Buat environment tanpa specify Python version (gunakan default)
conda create -n nama_env

# Buat environment dengan packages tertentu
conda create -n nama_env python=3.10 numpy pandas matplotlib
```
Penjelasan: Environment terpisah mencegah konflik antar dependencies project yang berbeda.

### Aktivasi Environment
```bash
conda activate nama_env
```
Penjelasan: Setelah aktivasi, semua package yang diinstall akan masuk ke environment ini.

### Deaktivasi Environment
```bash
conda deactivate
```
Penjelasan: Kembali ke base environment atau keluar dari conda environment.

### Melihat Daftar Environment
```bash
conda env list
```
atau
```bash
conda info --envs
```
Penjelasan: Menampilkan semua environment yang ada, tanda `*` menunjukkan environment yang aktif.

### Menghapus Environment
```bash
conda env remove -n nama_env
```
atau
```bash
conda remove -n nama_env --all
```
Penjelasan: Menghapus environment beserta semua package di dalamnya.

### Clone Environment
```bash
conda create -n env_baru --clone env_lama
```
Penjelasan: Membuat salinan environment dengan semua package yang sama.

### Rename Environment
```bash
# Conda tidak punya rename command, jadi harus clone lalu delete
conda create -n nama_baru --clone nama_lama
conda env remove -n nama_lama
```
Penjelasan: Workaround untuk rename karena conda tidak support rename langsung.

## Mengelola Packages

### Install Package
```bash
# Install single package
conda install numpy

# Install multiple packages
conda install numpy pandas matplotlib

# Install package versi tertentu
conda install numpy=1.24.0

# Install dari channel tertentu
conda install -c conda-forge fastapi
```
Penjelasan: Conda akan otomatis resolve dependencies yang dibutuhkan oleh package.

### Install Package di Environment Tertentu (Tanpa Aktivasi)
```bash
conda install -n nama_env numpy
```
Penjelasan: Berguna jika tidak ingin switch environment tapi perlu install package.

### Uninstall Package
```bash
conda remove numpy
```
atau
```bash
conda uninstall numpy
```
Penjelasan: Menghapus package dari environment yang sedang aktif.

### Update Package
```bash
# Update single package
conda update numpy

# Update semua package di environment
conda update --all
```
Penjelasan: Update package ke versi terbaru yang compatible dengan dependencies lainnya.

### Melihat Daftar Package
```bash
# List semua package di environment aktif
conda list

# List package dengan pattern tertentu
conda list numpy

# List package di environment tertentu
conda list -n nama_env
```
Penjelasan: Menampilkan package beserta versinya yang terinstall di environment.

### Search Package
```bash
# Search package di repository conda
conda search numpy

# Search dengan wildcard
conda search "numpy*"
```
Penjelasan: Mencari package yang tersedia untuk diinstall beserta versi yang ada.

## Export dan Import Environment

### Export Environment ke File
```bash
# Export dengan semua dependency detail
conda env export > environment.yml

# Export hanya package yang diinstall manual
conda env export --from-history > environment.yml

# Export ke file requirements.txt (untuk pip)
pip list --format=freeze > requirements.txt
```
Penjelasan: File YAML ini berisi daftar package dan versinya, berguna untuk sharing atau backup.

### Import/Create Environment dari File
```bash
# Buat environment dari file YAML
conda env create -f environment.yml

# Update environment yang sudah ada dari file YAML
conda env update -f environment.yml --prune
```
Penjelasan: Membuat environment yang identik di mesin lain atau restore dari backup.

### Contoh Format environment.yml
```yaml
name: myproject
channels:
  - conda-forge
  - defaults
dependencies:
  - python=3.10
  - numpy=1.24
  - pandas
  - pip
  - pip:
    - fastapi
    - uvicorn
```
Penjelasan: Format standar file environment conda yang mendefinisikan nama, channels, dan dependencies.

## Conda Channels

### Melihat Channels yang Aktif
```bash
conda config --show channels
```
Penjelasan: Channels adalah repository tempat conda mencari packages.

### Menambah Channel
```bash
conda config --add channels conda-forge
```
Penjelasan: conda-forge adalah community channel dengan packages tambahan yang tidak ada di default.

### Remove Channel
```bash
conda config --remove channels conda-forge
```

### Set Channel Priority
```bash
conda config --set channel_priority strict
```
Penjelasan: Strict priority membuat conda hanya ambil dari channel dengan prioritas tertinggi.

## Conda dengan Pip

### Install Pip di Conda Environment
```bash
conda install pip
```
Penjelasan: Best practice install pip via conda agar terintegrasi dengan baik.

### Install Package dengan Pip di Conda Environment
```bash
# Aktifkan environment dulu
conda activate nama_env

# Install via pip
pip install package_name
```
Penjelasan: Gunakan pip untuk package yang tidak tersedia di conda repository.

### Best Practice Conda + Pip
1. Install semua package yang bisa via conda terlebih dahulu
2. Install package via pip hanya jika tidak tersedia di conda
3. Hindari mixing conda dan pip untuk package yang sama

Penjelasan: Conda lebih baik handle dependencies, tapi pip punya lebih banyak packages.

## Cleaning dan Maintenance

### Membersihkan Cache dan Package yang Tidak Terpakai
```bash
# Bersihkan cache
conda clean --all

# Bersihkan package yang tidak terpakai
conda clean --packages

# Bersihkan index cache
conda clean --index-cache
```
Penjelasan: Menghemat disk space dengan menghapus file temporary dan cache yang tidak diperlukan.

### Cek Disk Usage Conda
```bash
conda clean --all --dry-run
```
Penjelasan: Melihat berapa space yang bisa dibebaskan tanpa benar-benar menghapus.

## Informasi dan Troubleshooting

### Melihat Info Conda
```bash
conda info
```
Penjelasan: Menampilkan informasi lengkap tentang instalasi conda, environment, dan konfigurasi.

### Melihat Lokasi Environment
```bash
conda env list
```
Penjelasan: Path lengkap environment ditampilkan di output command ini.

### Verifikasi Environment
```bash
# Cek package yang broken atau bermasalah
conda list --show-channel-urls

# Verify semua package
conda update --all --dry-run
```
Penjelasan: Berguna untuk troubleshooting masalah dependencies atau konflik package.

### Reset Conda ke Default
```bash
# Backup dulu file .condarc
cp ~/.condarc ~/.condarc.backup

# Reset config
conda config --remove-key channels
conda config --set auto_activate_base false
```
Penjelasan: Mengembalikan konfigurasi conda ke setting default jika ada masalah.

## Tips dan Best Practices

### Disable Auto Activate Base Environment
```bash
conda config --set auto_activate_base false
```
Penjelasan: Base environment tidak otomatis aktif saat buka terminal, lebih bersih.

### Membuat Environment untuk Project Tertentu
```bash
# Biasanya di root project
conda create -n myproject python=3.10
conda activate myproject
conda install package1 package2
conda env export > environment.yml
```
Penjelasan: Setiap project punya environment terpisah untuk isolasi dependencies.

### Jupyter Notebook dengan Conda Environment
```bash
# Install ipykernel di environment
conda activate nama_env
conda install ipykernel

# Daftarkan kernel ke Jupyter
python -m ipykernel install --user --name=nama_env --display-name "Python (nama_env)"

# Jalankan Jupyter
jupyter notebook
```
Penjelasan: Membuat conda environment bisa dipilih sebagai kernel di Jupyter Notebook.

### Cek Python Path di Environment
```bash
# Aktifkan environment
conda activate nama_env

# Cek path Python yang digunakan
which python

# Atau
python -c "import sys; print(sys.executable)"
```
Penjelasan: Memastikan Python yang digunakan adalah dari conda environment yang aktif.

## Integrasi Conda dengan Systemd (Linux)

Untuk menjalankan aplikasi Python dengan conda environment sebagai systemd service, gunakan template berikut:

```ini
[Unit]
Description=My Python App (Conda)
After=network.target

[Service]
User=youruser
Group=youruser
WorkingDirectory=/path/to/app
ExecStart=/bin/bash -c 'source /home/youruser/miniconda3/etc/profile.d/conda.sh && conda activate myenv && exec python app.py'
Restart=always

[Install]
WantedBy=multi-user.target
```
Penjelasan: Source conda.sh diperlukan agar conda command tersedia di systemd environment.

## Command Reference Cepat

```bash
# Environment Management
conda create -n myenv python=3.10    # Buat environment
conda activate myenv                  # Aktivasi environment
conda deactivate                      # Deaktivasi environment
conda env list                        # List environment
conda env remove -n myenv            # Hapus environment

# Package Management
conda install package                 # Install package
conda remove package                  # Hapus package
conda update package                  # Update package
conda list                           # List packages
conda search package                 # Search package

# Export/Import
conda env export > env.yml           # Export environment
conda env create -f env.yml          # Import environment

# Maintenance
conda clean --all                    # Bersihkan cache
conda update conda                   # Update conda
conda info                          # Info conda
```
Penjelasan: Daftar command yang paling sering digunakan untuk referensi cepat.