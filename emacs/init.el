;; emacs settings
;; Added by Package.el.  This must come before configurations of
;; installed packages.  Don't delete this line.  If you don't want it,
;; just comment it out by adding a semicolon to the start of the line.
;; You may delete these explanatory comments.

;; package list
(require 'package)
(add-to-list 'package-archives '("melpa-stable" . "https://stable.melpa.org/packages/") t)
(add-to-list 'package-archives '("melpa" . "https://melpa.org/packages/") t)

;; load shared packages
(add-to-list 'load-path "/usr/local/share/emacs/site-lisp")

(custom-set-variables
 ;; custom-set-variables was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 '(ag-highlight-search t)
 '(ag-reuse-buffers 'nil)
 '(ag-reuse-window 'nil)
 '(flycheck-checker-error-threshold 1000)
 '(init-loader-show-log-after-init 'error-only)
 '(package-selected-packages
   '(magit ccls use-package company lsp-ui lsp-mode gnu-elpa-keyring-update yaml-mode)))
(custom-set-faces
 ;; custom-set-faces was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 '(default ((t (:inherit nil :stipple nil :background "black" :foreground "white" :inverse-video nil :box nil :strike-through nil :overline nil :underline nil :slant normal :weight normal :height 113 :width normal :foundry "unknown" :family "DejaVu Sans Mono"))))
 '(font-lock-function-name-face ((t (:foreground "cyan"))))
 '(minibuffer-prompt ((t (:foreground "color-33"))))
 '(term-color-blue ((t (:background "deep sky blue" :foreground "deep sky blue")))))

(unless load-file-name
  (cd (getenv "HOME")))

;; use init-loader
(require 'init-loader)
(when load-file-name
  (setq-default user-emacs-directory (file-name-directory load-file-name)))
(init-loader-load (concat user-emacs-directory "conf"))
