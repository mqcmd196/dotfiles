;; C/C++
(use-package ccls
  :hook ((c-mode c++-mode objc-mode) .
         (lambda () (require 'ccls) (lsp)))
  :custom
  (ccls-executable (concat (getenv "HOME") "/dotfiles/emacs/ccls/Release/ccls"))
  (ccls-sem-highlight-method 'font-lock)
  :config
  (setq lsp-prefer-flymake nil)
  )
  ;; (ccls-use-default-rainbow-sem-highlight)
