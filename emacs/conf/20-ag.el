(require 'ag)
(custom-set-variables
 '(ag-highlight-search t)  ; Highlight search word
 '(ag-reuse-window 'nil)   ; use current window for search
 '(ag-reuse-buffers 'nil)) ; use current buffer for search
(require 'wgrep-ag)
(autoload 'wgrep-ag-setup "wgrep-ag")
(add-hook 'ag-mode-hook 'wgrep-ag-setup)
;; press r to move to edit mode
;; C-x C-s save, C-x C-k not save
(define-key ag-mode-map (kbd "r") 'wgrep-change-to-wgrep-mode)
