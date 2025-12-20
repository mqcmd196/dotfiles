FROM debian:testing

ENV DEBIAN_FRONTEND=noninteractive

# For manage user
RUN apt update -qq && \
    apt upgrade -y -qq && \
    apt install -y -qq --no-install-recommends \
    ansible apt git software-properties-common sudo

WORKDIR /home/$USERNAME/dotfiles
COPY non-sudoer ./non-sudoer
COPY roles ./roles
COPY setup_sudoer.yml .
RUN ansible-playbook setup_sudoer.yml -K
COPY tests ./tests
RUN ./tests/test_emacs.sh
RUN ./tests/test_shell.sh
