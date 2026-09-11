FROM debian:testing

ENV DEBIAN_FRONTEND=noninteractive

# For manage user
RUN apt update -qq && \
    apt upgrade -y -qq && \
    apt install -y -qq --no-install-recommends \
    ansible apt git sudo

WORKDIR /root/dotfiles
COPY non-sudoer ./non-sudoer
COPY roles ./roles
COPY prompts ./prompts
COPY setup_sudoer.yml .
RUN ansible-playbook setup_sudoer.yml
COPY tests ./tests
RUN rm -rf /var/lib/apt/lists/*
