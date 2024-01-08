sudo apt install -y git curl \
    python3 \
    python3-pip \
    libbz2-dev \
    libffi-dev \
    lzma liblzma-dev \
    python-tk python3-tk tk-dev \
    make build-essential libssl-dev zlib1g-dev \
    libreadline-dev libsqlite3-dev wget llvm libncurses5-dev libncursesw5-dev \
    xz-utils python-openssl python3-pil


if  ! which pyenv ; then
    curl https://pyenv.run | bash
    export PATH="$HOME/.pyenv/bin:$PATH" && eval "$(pyenv init --path)" && echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n eval "$(pyenv init -)"\nfi' >> ~/.bashrc
    source ~/.bashrc
fi

pyenv install 3.8.0
pyenv local 3.8.0
pip3 install --upgrade pip
cd lib/gym-duckietown
pip3 install -r requirements.txt
pip3 install -e .
pip3 install pyglet==1.5.0
