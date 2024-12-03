


# Installation

1. Create a new Anaconda environment: `conda create -n mm python=3.8`

2. `conda activate mm`

3. `pip3 install -r requirements.txt`

4. `export PYTHONPATH="$PYTHONPATH:<path to Mobile-Manipulation repo>"`

# Run

`python simulation/main.py`
`click 'u' for bot to collect the chosen obj`
We have 4 demo videos
1. Full path video shows the robot successfully planning a path and avoiding collisons to reach across the room and grasp an object.
2. The other three videos showcase the robot successfully planning a path and picking three objects (red, blue and white cups) and dropping them in end location. 


# References

PyBullet Documentation: https://pybullet.org/wordpress/index.php/forum-2/.

You can also use ChatGPT or Cursor to get the API of PyBullet.