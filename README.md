# 642RlFinal_bright_hu Repository

This project is based on four repositories, therefore only the newly created and modified code are uploaded to this repo.

**Note**: The disk image and trace files are exceptionally large and are not hosted in this repository. They can be obtained by requesting via email.

## Setup Instructions

### jobfile.sh
- Script to run Pythia and ChampSim

### gem5 Folder
- Clone the official gem5 repository: [gem5 repository](https://github.com/gem5/gem5).
- After cloning, copy all the files from this repository's `gem5` folder into the `gem5/src/cpu/o3/probe` directory in your local gem5 repository.

### System Folder and run_sim.py
- The `system` folder along with the `run_sim.py` script are utilized to configure a two-machine system within gem5. This setup includes both a client and a server.
