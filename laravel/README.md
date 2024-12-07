# Cheat Sheet Laravel
## Instalasi Laravel
Coming Soon
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
- Membuat migrasi dengan nama khusus
  ```
  php artisan make:migration create_table_name_table
  ```
## Seeding Database
- Menjalankan Seeder default (DatabaseSeeder)
  ```
  php artisan db:seed
  ```
- Menjalankan seeder tertentu
  ```
  php artisan db:seed --class=<NamaSeeder>
  ```
- Menjalankan migrasi dan seeder bersamaan
  ```
  php artisan migrate --seed
  ```
- Membuat seeder baru
  ```
  php artisan make:seeder <NamaSeeder>
  ```
## Membuat Component
- Menambahkan component dengan class (dapat menerima inputan)
  ```
  php artisan make:component <NamaComponent>
  ```
- Menambahkan component tanpa class (hanya view)
  ```
  php artisan make:component <NamaComponent> --view
  ```
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
## Membuat Middleware
```
php artisan make:middleware <NamaMiddleware>
```
## Cache
- Menghapus cache view (dari file blade)
  ```
  php artisan view:clear
  ```
- Menghapus cache aplikasi (semua data cache aplikasi)
  ```
  php artisan cache:clear
  ```
- Menghapus konfigurasi cache (menghapus cache dari folder config)
  ```
  php artisan config:clear
  ```
