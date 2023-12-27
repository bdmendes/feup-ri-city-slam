sudo apt install git curl python3 python3-pip libbz2-dev

if  ! which pyenv ; then
    curl https://pyenv.run | bash
    export PATH="$HOME/.pyenv/bin:$PATH" && eval "$(pyenv init --path)" && echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n eval "$(pyenv init -)"\nfi' >> ~/.bashrc
    source ~/.bashrc
fi

pyenv install 3.8.0
pyenv local 3.8.0
pip install --upgrade pip
cd lib/gym-duckietown
pip install -e .
pip install pyglet==1.5.0
