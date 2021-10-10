(r>equire 'helm-mode)
(require 'helm-config)
(setq helm-buffer-max-length 40)

;; helm git
(require 'helm-git-files)
(defun my-helm-git-files ()
  "`helm' for opening files managed by Git."
  (interactive)
  (helm-other-buffer `(helm-git-files:modified-source
		       helm-git-files:all-source
		       helm-git-files:untracked-source
		       ,@(helm-git-files:submodule-sources
			  '(modified untracked all)))
		     "*helm for git files*"))

(require 'helm-ls-git)
;; helm for files
(setq helm-for-files-preferred-list
      '(helm-source-ls-git
        helm-source-recentf
        helm-source-find-files
        helm-source-file-cache
        helm-source-files-in-current-dir
        helm-source-locate))

;; helm-source-ls-git is nil until helm-ls-git-ls is invoked once. Initialize here.
(setq helm-source-ls-git-status
      (helm-ls-git-build-git-status-source)
      helm-source-ls-git
      (helm-ls-git-build-ls-git-source)
      helm-source-ls-git-buffers
      (helm-ls-git-build-buffers-source))

;; helm ag
(require 'helm-ag)
(setq helm-ag-base-command "ag --nocolor --nogroup -i")
(setq helm-ag-insert-at-point 'symbol)
(setq helm-ag-source-type 'file-line)

;; helm-gtags
(require 'helm-gtags)
(setq helm-gtags-auto-update t)
(setq helm-gtags-read-only t)

(put 'helm-show-kill-ring 'delete-selection t) ; helm-show-kill-ring with delete selection mode

;; don't use helm for these commands
(add-to-list 'helm-completing-read-handlers-alist '(find-file-read-only . nil))
