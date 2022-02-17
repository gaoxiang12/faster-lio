# coding="utf8"
import sys
import pandas as pd
from matplotlib import pyplot as plt


def remove_outlier(x):
    q1 = x.quantile(0.25)
    q3 = x.quantile(0.75)
    m = (x <= q3 + (q3 - q1) * 1.5).all(axis=1)
    return x[m]


def time_plot(log_file):
    # read time log
    # log_file = "20130110.time.log"
    df = pd.read_csv(log_file)
    print("Read " + str(df.shape[0]) + " rows from " + log_file)
    # compute average
    for c in df:
        x = df[c]
        x = x[x.notna()]  # remove nan
        fmt = "%-35s: num=%d, ave=%f, std=%f, max=%f, min=%f"
        print(fmt % (c, len(x), x.mean(), x.std(), x.max(), x.min()))
    # plot
    c = [" Laser Mapping Single Run", " IEKF Solve and Update"]
    x = df[c]
    x = x[x.apply(pd.notna).all(axis=1)]  # remove nan
    x = remove_outlier(x)
    y = x.rolling(7, center=True).mean()  # sliding average, window size = 7
    fig = plt.figure(num=log_file)
    # _ = plt.plot(x)
    # fig = plt.figure(num=log_file + "(moving average = 7)")
    _ = plt.plot(y)
    plt.legend(y.columns)
    plt.ylabel("time/ms")
    plt.xlabel("laser scan")
    plt.savefig(log_file.replace(".log", ".png"))
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python2 plot_time.py 20120110.time.log")
    else:
        log_file = sys.argv[1]
        time_plot(log_file)
