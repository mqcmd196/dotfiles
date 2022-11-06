#
# Author: Yoshiki Obinata
#

ANSIBLEPB = ansible-playbook

SUDOER_FILE = setup_sudoer.yml

sudoer:
	$(ANSIBLEPB) $(SUDOER_FILE) -K
