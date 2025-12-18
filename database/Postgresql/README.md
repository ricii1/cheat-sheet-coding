# PostgreSQL Cheat Sheet

## Instalasi PostgreSQL

### Ubuntu/Debian
```bash
# Update package list
sudo apt update

# Install PostgreSQL
sudo apt install postgresql postgresql-contrib

# Cek versi yang terinstall
psql --version
```
Penjelasan: Menginstall PostgreSQL beserta tools tambahan yang berguna untuk manajemen database.

### CentOS/RHEL/Fedora
```bash
# Install PostgreSQL
sudo dnf install postgresql-server postgresql-contrib

# Initialize database
sudo postgresql-setup --initdb

# Cek versi yang terinstall
psql --version
```
Penjelasan: Untuk distro berbasis Red Hat, perlu inisialisasi database setelah install.

### macOS (menggunakan Homebrew)
```bash
# Install PostgreSQL
brew install postgresql

# Cek versi yang terinstall
psql --version
```
Penjelasan: Homebrew adalah package manager populer untuk macOS.

## Mengelola Service PostgreSQL

### Start PostgreSQL Service
```bash
# Ubuntu/Debian
sudo systemctl start postgresql

# macOS (Homebrew)
brew services start postgresql
```
Penjelasan: Memulai service PostgreSQL agar database dapat diakses.

### Stop PostgreSQL Service
```bash
# Ubuntu/Debian
sudo systemctl stop postgresql

# macOS (Homebrew)
brew services stop postgresql
```
Penjelasan: Menghentikan service PostgreSQL.

### Restart PostgreSQL Service
```bash
# Ubuntu/Debian
sudo systemctl restart postgresql

# macOS (Homebrew)
brew services restart postgresql
```
Penjelasan: Restart service PostgreSQL, biasanya setelah mengubah konfigurasi.

### Enable PostgreSQL pada Boot
```bash
# Ubuntu/Debian
sudo systemctl enable postgresql
```
Penjelasan: Membuat PostgreSQL otomatis berjalan saat sistem boot.

### Cek Status PostgreSQL Service
```bash
# Ubuntu/Debian
sudo systemctl status postgresql

# macOS (Homebrew)
brew services list
```
Penjelasan: Melihat apakah PostgreSQL sedang berjalan atau tidak.

## Konfigurasi Awal PostgreSQL

### Masuk ke PostgreSQL sebagai User Postgres
```bash
sudo -i -u postgres
psql
```
Penjelasan: User default `postgres` dibuat otomatis saat instalasi dan memiliki hak akses penuh.

### Keluar dari PostgreSQL
```sql
\q
```
atau tekan `Ctrl+D`

### Mengubah Password User Postgres
```bash
# Masuk ke PostgreSQL
sudo -u postgres psql

# Ubah password di dalam psql
ALTER USER postgres PASSWORD 'password_baru';

# Keluar
\q
```
Penjelasan: Mengatur password untuk user postgres agar lebih aman.

## Membuat User dan Database Baru

### Membuat User Baru
```bash
# Dari terminal (tanpa masuk psql)
sudo -u postgres createuser --interactive --pwprompt

# Atau dari dalam psql
CREATE USER nama_user WITH PASSWORD 'password';
```
Penjelasan: User baru digunakan untuk memisahkan hak akses antar aplikasi atau developer.

### Membuat Database Baru
```bash
# Dari terminal
sudo -u postgres createdb nama_database

# Atau dari dalam psql
CREATE DATABASE nama_database;
```
Penjelasan: Setiap aplikasi biasanya membutuhkan database terpisah.

### Memberikan Hak Akses User ke Database
```sql
# Dari dalam psql
GRANT ALL PRIVILEGES ON DATABASE nama_database TO nama_user;
```
Penjelasan: User perlu diberi hak akses eksplisit untuk mengakses database tertentu.

## Koneksi ke Database

### Koneksi dari Terminal
```bash
# Koneksi sebagai user postgres ke database tertentu
sudo -u postgres psql -d nama_database

# Koneksi sebagai user lain
psql -U nama_user -d nama_database -h localhost
```
Penjelasan: `-U` untuk user, `-d` untuk database, `-h` untuk host.

### Koneksi String untuk Aplikasi
```
postgresql://username:password@localhost:5432/nama_database
```
Penjelasan: Format connection string yang umum digunakan di berbagai framework.

## Perintah Dasar PostgreSQL

### Melihat Daftar Database
```sql
\l
```
atau
```sql
\list
```

### Melihat Daftar User/Role
```sql
\du
```

### Pindah ke Database Lain
```sql
\c nama_database
```

### Melihat Daftar Tabel
```sql
\dt
```

### Melihat Struktur Tabel
```sql
\d nama_tabel
```

### Menjalankan Query dari File
```bash
psql -U username -d database_name -f file.sql
```

## Backup dan Restore

### Backup Database
```bash
# Backup single database
pg_dump -U postgres nama_database > backup.sql

# Backup dengan format custom (lebih efisien)
pg_dump -U postgres -Fc nama_database > backup.dump

# Backup semua database
pg_dumpall -U postgres > all_databases.sql
```
Penjelasan: Backup rutin penting untuk menghindari kehilangan data.

### Restore Database
```bash
# Restore dari SQL file
psql -U postgres nama_database < backup.sql

# Restore dari custom format
pg_restore -U postgres -d nama_database backup.dump
```
Penjelasan: Restore digunakan untuk mengembalikan data dari backup.

## Konfigurasi File

### Lokasi File Konfigurasi
```bash
# Ubuntu/Debian
/etc/postgresql/[version]/main/postgresql.conf
/etc/postgresql/[version]/main/pg_hba.conf

# CentOS/RHEL
/var/lib/pgsql/data/postgresql.conf
/var/lib/pgsql/data/pg_hba.conf
```

### Mengizinkan Koneksi Remote (pg_hba.conf)
Tambahkan baris berikut di `pg_hba.conf`:
```
# IPv4 local connections:
host    all             all             0.0.0.0/0               md5
```
Penjelasan: Mengizinkan koneksi dari IP manapun dengan autentikasi password.

### Mengubah Listen Address (postgresql.conf)
```
listen_addresses = '*'
```
Penjelasan: Membuat PostgreSQL listen di semua network interface.

**Jangan lupa restart service setelah mengubah konfigurasi!**

## Tips dan Troubleshooting

### Cek Port PostgreSQL
```bash
sudo netstat -plunt | grep postgres
```
Default port: 5432

### Melihat Koneksi Aktif
```sql
SELECT * FROM pg_stat_activity;
```

### Kill Koneksi yang Mengganggu
```sql
SELECT pg_terminate_backend(pid) 
FROM pg_stat_activity 
WHERE datname = 'nama_database' AND pid <> pg_backend_pid();
```
Penjelasan: Berguna saat ada koneksi yang tidak bisa ditutup dan menghalangi operasi database.

## Integrasi dengan Laravel

### Instalasi Driver PostgreSQL untuk Laravel
```bash
# Install PHP PostgreSQL extension (jika belum ada)
sudo apt install php-pgsql

# Restart PHP-FPM atau Apache/Nginx
sudo systemctl restart php8.1-fpm
```
Penjelasan: Laravel memerlukan extension `pgsql` dan `pdo_pgsql` untuk bisa terhubung ke PostgreSQL.

### Konfigurasi File .env Laravel
Edit file `.env` di root project Laravel:
```env
DB_CONNECTION=pgsql
DB_HOST=127.0.0.1
DB_PORT=5432
DB_DATABASE=nama_database
DB_USERNAME=nama_user
DB_PASSWORD=password_user
```
Penjelasan: Ubah `DB_CONNECTION` dari `mysql` ke `pgsql`, lalu sesuaikan kredensial database PostgreSQL yang sudah dibuat.

### Konfigurasi Database (Opsional)
File `config/database.php` sudah memiliki konfigurasi default PostgreSQL:
```php
'pgsql' => [
    'driver' => 'pgsql',
    'url' => env('DATABASE_URL'),
    'host' => env('DB_HOST', '127.0.0.1'),
    'port' => env('DB_PORT', '5432'),
    'database' => env('DB_DATABASE', 'forge'),
    'username' => env('DB_USERNAME', 'forge'),
    'password' => env('DB_PASSWORD', ''),
    'charset' => 'utf8',
    'prefix' => '',
    'prefix_indexes' => true,
    'search_path' => 'public',
    'sslmode' => 'prefer',
],
```
Penjelasan: Konfigurasi ini biasanya tidak perlu diubah, cukup atur di file `.env` saja.

### Test Koneksi Database
```bash
# Jalankan migration untuk test koneksi
php artisan migrate

# Atau test dengan tinker
php artisan tinker
>>> DB::connection()->getPdo();
```
Penjelasan: Jika migration berhasil atau tinker menampilkan object PDO, berarti koneksi sudah berhasil.

### Contoh Lengkap Setup PostgreSQL untuk Laravel
```bash
# 1. Buat database dan user
sudo -u postgres psql
CREATE DATABASE laravel_db;
CREATE USER laravel_user WITH PASSWORD 'laravel_password';
GRANT ALL PRIVILEGES ON DATABASE laravel_db TO laravel_user;
\q

# 2. Update .env Laravel
# DB_CONNECTION=pgsql
# DB_HOST=127.0.0.1
# DB_PORT=5432
# DB_DATABASE=laravel_db
# DB_USERNAME=laravel_user
# DB_PASSWORD=laravel_password

# 3. Clear config cache dan migrate
php artisan config:clear
php artisan migrate
```
Penjelasan: Langkah lengkap dari membuat database PostgreSQL sampai menjalankan migration Laravel.
