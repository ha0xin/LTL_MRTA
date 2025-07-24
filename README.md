# Multi-robot Task Allocation and Planning under Linear Temporal Logic Specifications
这是一个LTL_MRTA的修改版本，原仓库地址为：https://github.com/XushengLuo92/LTL_MRTA

# Install
The code is tesed using Python 3.10.17.
## Step 1: install dependencies via uv
```bash
uv sync
uv pip install -e .
```
## Step 2: install ltl2ba
Download the software `LTL2BA` from this [link](http://www.lsv.fr/~gastin/ltl2ba/index.php), and follow the instructions to generate the exectuable `ltl2ba` and then copy it into the root folder `LTL_MRTA`.

## Step 3: install gurobi
Follow the instructions on the [Gurobi website](https://www.gurobi.com) to install the Gurobi optimizer.

# Usage
## Run the example