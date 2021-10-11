;; C/C++
(use-package ccls
  :custom
  (ccls-executable (concat (getenv "HOME") "/dotfiles/emacs/ccls/Release/ccls"))
  (ccls-sem-highlight-method 'font-lock)
  ;; (ccls-use-default-rainbow-sem-highlight)
  :hook ((c-mode c++-mode objc-mode) .
         (lambda () (require 'ccls) (lsp))))
