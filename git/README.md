# Cheat Sheet Git
## Daftar Isi
- [Cheat Sheet Git](#cheat-sheet-git)
  - [Daftar Isi](#daftar-isi)
  - [Konfigurasi](#konfigurasi)
  - [Inisialisasi Repository](#inisialisasi-repository)
  - [Commit dan Perubahan](#commit-dan-perubahan)
  - [Branch](#branch)
  - [Merge branch](#merge-branch)
  - [Remote Repository](#remote-repository)
  - [Stashing](#stashing)
  - [Cleaning](#cleaning)
  - [Logging](#logging)
  - [Reset](#reset)
  - [Revert](#revert)
  - [Rebase](#rebase)
  - [Submodule](#submodule)
  - [Debugging](#debugging)
## Konfigurasi
Pengaturan awal Git seperti username dan email.
- Mengisi username dan email secara global
  ```sh
  git config --global user.name "<nama_user>"
  git config --global user.email "<email_user>"
  ```
- Mengisi username dan email
  ```sh
  git config user.name "<nama_user>"
  git config user.email "<email_user>"
  ```
## Inisialisasi Repository
Membuat repository Git baru.
- Membuat repository baru
  ```
  git init
  ```
- Mengkloning repository
  ```
  git clone <link_repo>
  ```
## Commit dan Perubahan
Menyimpan perubahan ke dalam repository.
- Melihat status perubahan
  ```
  git status
  ```
- Menambahkan file ke staging
  ```
  git add <file>
  ```
- Melakukan commit
  ```sh
  git commit -m "Pesan commit"
  ```
## Branch
Membuat dan mengelola cabang (branch) dalam repository.
- Membuat branch baru
  ```
  git branch <nama_branch>
  ```
- Pindah branch
  ```
  git checkout <nama_branch>
  ```
  atau
  ```
  git switch <nama_branch>
  ```
- Membuat dan pindah branch
  ```
  git branch -b <nama_branch>
  ```
- Mengubah nama branch tanpa force (akan error jika nama branch baru sudah dipakai)
  ```
  git branch -m <nama_branch_baru>
  ```
- Mengubah nama branch force (tidak error jika nama branch baru sudah dipakai)
  ```
  git branch -M <nama_branch_baru>
  ```
- Menghapus branch soft delete (hanya menggabungkan jika perubahan sudah digabungkan)
  ```
  git branch -d <nama_branch>
  ```
- Menghapus branch force delete (memaksa penghapusan meskipun ada peruubahan yang belum digabungkan)
  ```
  git branch -D <nama_branch>
  ```
- Menampilkan branch yang ada di remote
  ```
  git branch -r
  ```
- Menampilkan semua branch
  ```
  git branch -a
  ```
## Merge branch
Menggabungkan perubahan dari satu branch ke branch lain.
- Menggabungkan branch
  ```
  git merge
  ```  
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