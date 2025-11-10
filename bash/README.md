# Cheat Sheet Bash

## Systemd

Systemd adalah sistem init dan manajer layanan yang digunakan untuk mengelola proses di Linux. Berikut langkah-langkah membuat dan mengelola service dengan systemd:

### 1. Membuat File Service
Buat file unit di `/etc/systemd/system/nama-service.service` dengan isi seperti berikut:
```ini
[Unit]
Description=Deskripsi singkat service

[Service]
ExecStart=/path/to/script-atau-binary
Restart=always
User=username

[Install]
WantedBy=multi-user.target
```
File ini mendefinisikan bagaimana service dijalankan dan dikelola oleh systemd.

### 2. Reload Systemd
Setelah membuat atau mengubah file service, reload systemd agar perubahan terbaca:
```bash
sudo systemctl daemon-reload
```
Perintah ini memuat ulang konfigurasi systemd tanpa perlu reboot.

### 3. Menjalankan Service (Start)
```bash
sudo systemctl start nama-service
```
Memulai service secara manual.

### 4. Menghentikan Service (Stop)
```bash
sudo systemctl stop nama-service
```
Menghentikan service yang sedang berjalan.

### 5. Merestart Service (Restart)
```bash
sudo systemctl restart nama-service
```
Menghentikan lalu menjalankan ulang service.

### 6. Mengaktifkan Service saat Boot (Enable)
```bash
sudo systemctl enable nama-service
```
Membuat service otomatis berjalan saat sistem booting.

### 7. Menonaktifkan Service saat Boot (Disable)
```bash
sudo systemctl disable nama-service
```
Mencegah service berjalan otomatis saat booting.

## SCP
SCP digunakan untuk mengirim file antara remote dan local
- Mengirim file dari remote ke local
  ```
  scp -r user@remote_server:/path/to/remote/directory /local/directory
  ```
