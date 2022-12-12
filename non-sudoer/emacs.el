;; C-h to delete
(global-set-key "\C-h" 'delete-backward-char)

(setq initial-scratch-message 'nil)

(setq whitespace-space-regexp "\\(\u3000+\\)")

(show-paren-mode 1)

(setq-default tab-width 4 indent-tabs-mode nil)

;; yaml-mode, nxml in ros
(when (require 'yaml-mode nil t)
  (add-to-list 'auto-mode-alist '("\\.yml$" . yaml-mode)))
(when (require 'nxml-mode nil t)
  (add-to-list 'auto-mode-alist '("\\.launch$" . nxml-mode))
  (add-to-list 'auto-mode-alist '("\\.xml$" . nxml-mode)))
