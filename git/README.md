# Cheat Sheet Git
## Konfigurasi
Pengaturan awal Git seperti username dan email.
## Inisialisasi Repository
Membuat repository Git baru.
## Commit dan Perubahan
Menyimpan perubahan ke dalam repository.
## Branch
Membuat dan mengelola cabang (branch) dalam repository.
## Merge branch
Menggabungkan perubahan dari satu branch ke branch lain.
  - Merge branch remote dengan local dengan allow-unrelated-histories
    ```
    git merge origin/main --allow-unrelated-histories
    ```
## Remote Repository
Mengelola repository yang di-hosting di server remote.
## Stashing
Menyimpan perubahan sementara tanpa melakukan commit.
## Cleaning
Menghapus file yang tidak diperlukan dari working directory.
## Logging
Melihat riwayat commit dalam repository.
## Reset
Mengembalikan repository ke keadaan sebelumnya.
## Revert
Membatalkan perubahan dengan membuat commit baru.
## Rebase
Menggabungkan perubahan dari satu branch ke branch lain dengan cara yang lebih bersih.

## Submodule
Mengelola repository Git di dalam repository lain.
- Menambahkan submodule
  ```
  git submodule add <url_repo>
  ```
- Mengupdate submodule
  git submodule update --init --recursive
## Debugging
Mendiagnosa dan memperbaiki masalah dalam repository.
- Melihat perubahan dalam file
  git diff
- Melihat siapa yang mengubah baris tertentu
  ```
  git blame <nama_file>
  ```
- Mengembalikan branch yang terhapus
  ```
  git reflog
  ```