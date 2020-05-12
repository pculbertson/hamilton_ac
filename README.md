# hamilton_ac
This repository contains the source code for [Decentralized Adaptive Control for Collaborative Manipulation of Rigid Bodies](https://arxiv.org/pdf/2005.03153.pdf), by Preston Culbertson, Jean-Jacques Slotine, and Mac Schwager.

## Installation ##

This repository uses Julia-1.1 to implement the numerical simulations described in the paper, and Python 3.x to implement the experimental code, using [ROS](https://www.ros.org/).

You can install the Julia packages needed to run the simulations with the following script:
```
import Pkg;
for p in ["NLsolve, PyPlot,  Primes"]
    Pkg.add(p)
end
```

The experimental code runs on top of the Ouijabot platform and ROS package, which can be found [here.](https://github.com/StanfordMSL/ouijabot)
