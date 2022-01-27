;; lsp-mode, language server protocol
(require 'lsp-mode)
(require 'flycheck)
(use-package lsp-mode
  :commands lsp
  :config
  (setq gc-cons-threshold 100000000)
  (setq read-process-output-max (* 1024 1024))
  (setq lsp-completion-provider :capf)
  (setq lsp-idle-delay 0.500)
  (setq lsp-log-io nil) ; if set to true can cause a performance hit
  (setq lsp-diagnostic-provider :flycheck) ; use flycheck instead of flymake to improve the performance
  (setq lsp-signature-auto-activate t)
  (setq lsp-signature-doc-lines 1) ; restrict lines of lsp-signature doc showed at the bottom
  (setq lsp-prefer-flymake nil)
  )

;; lsp-ui
(require 'lsp-ui)
(setq lsp-ui-imenu-enable t)
(setq lsp-headerline-breadcrumb-enable nil)
