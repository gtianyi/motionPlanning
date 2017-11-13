#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def plot(df):
    pass


def main():
    #start(int), end(int), estimate(bool), alpha(int), beta(int)
    df = pd.read_csv('test.csv')
    print(df.columns)
    print(df.describe())

    # Accuracy of abstract estimate:
    accurate_estimate_count = sum(np.equal(df.estimate, df.alpha > df.beta))
    print("Accurate abstact estimates: {}".format(accurate_estimate_count))

    # Plot alpha/beta distributions
    df['expected'] = df.alpha / (df.beta + df.alpha)
    df.hist(column='expected', by='estimate')

    plt.show()


if __name__ == '__main__':
    main()

