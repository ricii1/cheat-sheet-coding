# Cheat Sheet Laravel
## Instalasi Laravel

## Memulai Suatu Project
- Membuat project dengan dependensi global
  ```
  laravel new <nama_project>
  ```
- Membuat project dengan dependensi local
  ```
  composer create-project --prefer-dist laravel/laravel <nama_project>
  ```
## Melihat Route
```
php artisan route:list
```
## Membuat Controller
- Membuat controller <br>
  ```
  php artisan make:controller <nama_controller>
  ```
- Membuat controller dengan resource (template) <br>
  ```
  php artisan make:controller <nama_controller> --resource
  ```
## Membuat Model
```
php artisan make:model <nama_model>
```
## Migrasi Database
- Menerapkan semua migrasi untuk pertama kali
  ```
  php artisan migrate
  ```
- Membatalkan migrasi terakhir
  ```
  php artisan migrate:rollback
  ```
- Menghapus semua migrasi
  ```
  php artisan migrate:reset
  ```
- Menjalankan migrate:reset dan migrate bersamaan
  ```
  php artisan migrate:refresh
  ```
- Migrate refresh dengan seeder
  ```
  php artisan migrate:refresh --seed
  ```
- Melihat status migrate
  ```
  php artisan migrate:status
  ```
- 
## Seeding Database
## Optimize aplikasi
Meningkatkan performa aplikasi dengan mengoptimalkan konfigurasi, route, dan view.
```
php artisan optimize
```
## Memulai Web Server
- Menjalankan secara default di ```http://localhost:8000```
  ```
  php artisan serve
  ```
- Menjalankan dengan mengatur port
  ```
  php artisan serve --port=<port>
  ```
- Menjalankan dengan membuka web server yang dapat diakses
  ```
  php artisan serve --host=0.0.0.0 --port=80
  ```
## Menjalankan Testing
```
php artisan test
```
