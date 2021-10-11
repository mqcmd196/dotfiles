;; Python
(require 'lsp-python-ms)
(add-hook 'hack-local-variables-hook
          (lambda ()
            (when (derived-mode-p 'python-mode)
              (require 'lsp-python-ms)
              (lsp)))) ; or lsp-deferred
;; (use-package lsp-python-ms
;;   :init
;;   (setq lsp-python-ms-auto-install-server t)
;;   :hook
;;   ((python-mode . (lambda ()
;;                     (require 'lsp-python-ms)
;;                     (lsp-deferred)))
;;    (flycheck-mode . (lambda ()
;;                       (flycheck-add-next-checker 'lsp 'python-flake8)
;;                       ))
;;    )
;;   )
(setq lsp-python-ms-auto-install-server t)
(lsp-register-custom-settings
 `(("python.analysis.cachingLevel" lsp-python-ms-cache)
   ("python.analysis.errors" lsp-python-ms-errors)
   ("python.analysis.warnings" lsp-python-ms-warnings)
   ("python.analysis.information" lsp-python-ms-information)
   ("python.analysis.disabled" lsp-python-ms-disabled)
   ("python.autoComplete.extraPaths" lsp-python-ms-extra-paths)
   ("python.analysis.autoSearchPaths" ,(<= (length lsp-python-ms-extra-paths) 0) t)))
