;; use fetch1075's tab settings
(setq-default indent-tabs-mode nil)
(setq-default tab-width 2
			  c-basic-offiset 2
			  js-indent-level 2)
(add-hook 'python-mode-hook '(lambda ()
							   (setq python-indent-offset 4)))
(add-hook 'c-mode-common-hook '(lambda ()
								 (c-set-style "linux")
								 (setq c-basic-offset tab-width)))
