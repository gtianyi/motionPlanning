import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from math import *


def load_data(*result_files):
    df = None

    for result_file in result_files:
        if df is None:
            df = pd.read_json(result_file)
        else:
            df.append(pd.read_json(result_file))

    return df


def calculate_relative_instance_time(df, baseline_algorithm):
    def add_reference_column(group):

        assert group[group.Algorithm == baseline_algorithm].shape[0] == 1
        baseline_time = group[group.Algorithm == baseline_algorithm].time.min()
        group['baseline_time'] = baseline_time
        return group

    df = df.groupby('instance').apply(add_reference_column)
    df['relative_time'] = (df.time - df.baseline_time) / df.baseline_time
    return df


def add_instance_column(df):
    df['instance'] = df.Domain + '_' + df.Seed.astype(str) + '_' + \
                     df.AgentMesh + '_' + df.EnvironmentMesh + '_' + \
                     df.EnvironmentBounds + '_' + df.Start + '_' + df.Goal
    df.instance = df.instance.str.replace(' ', '#')


def scatter_plot(algorithm_lhs, algorithm_rhs):
    return df.pivot(index='instance', columns='Planner')['time'].plot.scatter(algorithm_lhs, algorithm_rhs)


# df = load_data('results/data-all-alloneone.json')
df = load_data(
    'results/data-hnb-kcar&dcar-nog.json',
    'results/data-hnb-h&q&b-nog.json',
    'results/data-h&q&b-nog.json',
    'results/data-gust.json',
    'results/data-dcar&kcar-nog.json')

add_instance_column(df)
df = calculate_relative_instance_time(df, 'BEAST')

# sns.boxplot(x='EnvironmentMesh', y='time', hue='Planner', data=df)
# plt.show()
# plt.savefig('test.svg')


fig = plt.figure()
count = df.Domain.unique().size
c = int(ceil(sqrt(count)))
r = int(ceil(float(count) / float(c)))
# sns.set(font_scale = 0.8)
for i, d in enumerate(df.Domain.unique()):
    axcurr = fig.add_subplot(r, c, i + 1)
    box_plot = sns.boxplot(x='EnvironmentName',
                           y='relative_time',
                           hue='Algorithm',
                           ax=axcurr,
                           data=df.loc[df['Domain'] == d],
                           showfliers=False,
                           showmeans=True, meanline=True, notch=True, palette="Set3")
    box_plot.set_title(d)
    box_plot.set_xlabel('')
    box_plot.set_xticklabels(box_plot.get_xticklabels(), rotation=17)
    if i + 1 == count:
        box_plot.legend(loc=2, bbox_to_anchor=(1.05, 1))
    else:
        box_plot.legend_.remove()

plt.tight_layout()
plt.savefig('beast-bonus-halton-nog.png')
plt.show()

if __name__ == '__main__':
    print('done')
