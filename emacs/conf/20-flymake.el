(require 'flymake)
(with-eval-after-load 'flycheck
  (advice-add 'flycheck-eslint-config-exists-p :override (lambda() t)))
(setq flycheck-check-syntax-automatically '(save mode-enable))
