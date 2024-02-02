(setq initial-scratch-message 'nil)
(setq inhibit-startup-message 't)

;; loading themes
(add-to-list 'custom-theme-load-path "~/dotfiles/non-sudoer/themes/obinata-deeper-blue")
(add-to-list 'load-path "~/dotfiles/non-sudoer/themes/obinata-deeper-blue")
(setq custom-theme-directory "~/dotfiles/non-sudoer/themes/obinata-deeper-blue")
(add-to-list 'load-path "/tmp/emacs-one-themes")
(add-to-list 'custom-theme-load-path "/tmp/emacs-one-themes")

;; when light mode, use one-light. else use deeper-blue
(let* ((os-color-scheme (getenv "OS_COLOR_SCHEME")))
  (if (string= os-color-scheme "light")
      (load-theme 'one-light t)
    ;; use obinata-deeper-blue
    (load-theme 'obinata-deeper-blue t)
    ;; black background
    (set-background-color "#2E3436")
    ;; black background in terminal
    (defun on-after-init ()
      (unless (display-graphic-p (selected-frame))
        (set-face-background 'default "unspecified-bg" (selected-frame))))
    (add-hook 'window-setup-hook 'on-after-init)))

;; Use Cascadia Code by default if exists
(cond
 ((find-font (font-spec :name "Cascadia Code"))
  (set-frame-font "Cascadia Code-12")))

;; backup directory
(setq backup-directory-alist '((".*" . "~/tmp")))
(setq auto-save-file-name-transforms   '((".*" "~/tmp/" t)))

(setq whitespace-space-regexp "\\(\u3000+\\)")

(show-paren-mode 1)

(setq-default tab-width 4 indent-tabs-mode nil)
(setq-default c-basic-offset 4)

;; Insert parenthesis/brackets by pair
(electric-pair-mode 1)

;; yes -> y, no -> n
(defalias 'yes-or-no-p 'y-or-n-p)

;; yaml-mode, nxml in ros
(when (require 'yaml-mode nil t)
  (add-to-list 'auto-mode-alist '("\\.yml$" . yaml-mode)))
(add-to-list 'auto-mode-alist '("\.launch$" . nxml-mode))
(add-to-list 'auto-mode-alist '("\.test$" . nxml-mode))
(add-to-list 'auto-mode-alist '("manifest.xml" . nxml-mode))
(add-to-list 'auto-mode-alist '("\\.urdf" . xml-mode))
(add-to-list 'auto-mode-alist '("\\.xacro" . xml-mode))
(add-to-list 'auto-mode-alist '("\\.msg\\'" . gdb-script-mode))
(add-to-list 'auto-mode-alist '("\\.srv\\'" . gdb-script-mode))
(add-to-list 'auto-mode-alist '("\\.action\\'" . gdb-script-mode))
(font-lock-add-keywords 'gdb-script-mode                        '(("\\<\\(bool\\|byte\\|int8\\|uint8\\|int16\\|uint16\\|int32\\|uint32\\|int64\\|uint64\\|float32\\|float64\\|string\\|time\\|duration\\)\\>" . font-lock-builtin-face)) 'set)

;; Keybinds
;; do not copy when M-DEL and C-backspace
(defun delete-word (arg)
  (interactive "p")
  (delete-region (point) (progn (forward-word arg) (point))))
(defun backward-delete-word (arg)
  (interactive "p")
  (delete-word (- arg)))
(global-set-key (read-kbd-macro "<M-DEL>") 'backward-delete-word) ;; not kill-ring with M-DEL
(global-set-key (read-kbd-macro "<C-backspace>") 'delete-word) ;; not kill-ring with M-d

;; C-h to delete
(global-set-key "\C-h" 'delete-backward-char)
;; call ibuffer in current window
(global-set-key (kbd "C-x C-b") 'ibuffer)
