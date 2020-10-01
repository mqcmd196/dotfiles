(add-to-list 'load-path "/opt/ros/melodic/share/euslime")
(require 'euslime-config)
(setq inferior-euslisp-program "roseus")
(slime-setup '(slime-fancy slime-banner slime-repl-ansi-color))
