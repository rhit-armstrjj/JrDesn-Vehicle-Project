# JrDesn-Vehicle-Project

ECE362 Code for 2023

Assumptions about the project:

- The bluetooth communications will always be in range, and will therefore, have no drop.

## Arduino

### Setting Up

1. Install the Arduino CLI & Arduino Extension for VSCode.
2. Install Python 3.9+
3. Install PuTTY if you want to test
4. Activate the Venv terminal if you want to get rid of the erorr squiggles.

### Arduino Usage

To upload code & start logging, run `Ctrl + Shift + P` and press `Tasks: Run task`, then select `Upload & Start Logging`.
After uploading there is a ten second window for you to unplug the car from your computer for logging to work. Then, there's a 30 second window for the terminal to start logging.

## Python

This project uses `venv` for it's dependencies. To enter the virtual environment, in windows, run the following command: `.\python\Scripts\activate`.
Once activated, there should be a little `(python)` at the start of your terminal.
If you don't see that, you might want to consult this [StackOverflow](https://stackoverflow.com/questions/18713086/virtualenv-wont-activate-on-windows) q&a.

On your first install, run `pip install -r ./python/requirements.txt`

To leave the virtual environment, just run `deactivate`.

Ideally, everything should be in the repository already so you do not need to install anything (Yay!).

### Python Usage

To run the program you can just run `python ./python/main.py`. However, if you are editing CSS, I would recommend using `textual run ./python/main.py` because that allows for live-viewing changes.

If you want to debug textual, run `textual console` in one terminal, and then open in another terminal `textual run --dev ./python/main.py`. This will give you a live console from Textual.

## Technologies Used

- [Textual](https://textual.textualize.io/)
- [MessagePack](https://msgpack.org/)
- [Matplotlib](https://matplotlib.org/stable/index.html#)
