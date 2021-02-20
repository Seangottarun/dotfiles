# Dotfiles

Various dotfile configuration files for various terminal applications. The
[dotbot](https://github.com/anishathalye/dotbot) tool is used to bootstrap these
dotfiles for ease of use. Modify the `install.conf.yaml` file to change dotbot
settings.


# Making Local Customizations

You can make local customizations for some programs by editing these files:

- `vim` : `~/.vimrc_local`
- `zsh` / bash : `~/.shell_local_before` run first
- `zsh` : `~/.zshrc_local_before` run before .zshrc
- `zsh` : `~/.zshrc_local_after` run after .zshrc
- `zsh` / bash : `~/.shell_local_after` run last
- `git` : `~/.gitconfig_local`
- `hg` : `~/.hgrc_local`
- `tmux` : `~/.tmux_local.conf`

# Setup Ultisnips
Install sympy to vim python
1. Check that your vim supports python3 commands with `:version`. It should have a `+python3`
2. Check where your vim python stores its packages with `:python3 import site; print(site.getsitepackages())`
3. Go to that path and force pip to install to that directory `pip3 install -t . sympy --ignore-installed`

Note: this is only required for VIM installed through brew. Brew lists python3 as a dependency and so it will
also install python3. Sympy is not there by default, so you have to install it yourself.

Or on Ubuntu, you can do

```bash
env -i bash
sudo apt install python3-pip
pip3 install numpy sympy
```

Note: UltiSnips goes from $1 to $2 to $n and then goes to $0 ($0 is always last)
