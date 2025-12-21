(setq initial-scratch-message 'nil)
(setq inhibit-startup-message 't)
(menu-bar-mode -1)
(tool-bar-mode 0)
(load-theme 'deeper-blue)
(setq backup-directory-alist '((".*" . "~/tmp"))) ;; backup
(setq auto-save-file-name-transforms   '((".*" "~/tmp/" t))) ;; backup
(setq whitespace-space-regexp "\\(\u3000+\\)")
(show-paren-mode 1) ;; emphasize paren pair
(setq-default tab-width 4 indent-tabs-mode nil)
(setq-default c-basic-offset 4)
(electric-pair-mode 1) ;; Insert parenthesis/brackets by pair
(which-function-mode 1) ;; always show which function
(setq sgml-quick-keys 'close) ;; auto add closing tag
(defalias 'yes-or-no-p 'y-or-n-p) ;; yes -> y, no -> n
(cond
 ((find-font (font-spec :name "Cascadia Code"))
  (set-frame-font "Cascadia Code"))) ;; load cascadia code

;; folding
(add-hook 'prog-mode-hook #'hs-minor-mode)
(add-hook 'xml-mode-hook
          '(lambda ()
             (hs-minor-mode 1)))

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
(font-lock-add-keywords 'gdb-script-mode '(("\\<\\(bool\\|byte\\|int8\\|uint8\\|int16\\|uint16\\|int32\\|uint32\\|int64\\|uint64\\|float32\\|float64\\|string\\|time\\|duration\\)\\>" . font-lock-builtin-face)) 'set)

;; Keybinds
(global-set-key "\C-h" 'delete-backward-char) ;; C-h to delete
(global-set-key (kbd "C-x C-b") 'ibuffer) ;; call ibuffer in current window
