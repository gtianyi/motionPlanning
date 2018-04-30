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
            df2 = pd.read_json(result_file)
            df = pd.concat([df,df2], axis = 0, ignore_index = True)

    return df

def get_lim(subdf):
    max=0
    for alg in subdf.Algorithm.unique():
        dfa = subdf.loc[subdf['Algorithm'] == alg]
        quant = dfa.time.quantile(.8)
        if max < quant:
            max = quant
    return max

def calculate_relative_instance_time(df, baseline_algorithm):
    def add_reference_column(group):

        assert group[group.Algorithm == baseline_algorithm].shape[0] == 1
        baseline_time = group[group.Algorithm == baseline_algorithm].time.max()
        group['baseline_time'] = baseline_time
        return group

    df = df.groupby('instance').apply(add_reference_column)
    df['relative_time'] = (df.time - df.baseline_time)
    return df


def add_instance_column(df):
    df['instance'] = df.Domain + '_' + df.Seed.astype(str) + '_' + \
                     df.AgentMesh + '_' + df.EnvironmentMesh + '_' + \
                     df.EnvironmentBounds + '_' + df.Start + '_' + df.Goal
    df.instance = df.instance.str.replace(' ', '#')

df = load_data(
   # 'results/data-all-alloneone.json', 
#    'results/data-hnb-kcar&dcar-nog.json',
#    'results/data-hnb-h&q&b-nog.json',
    # 'results/data-beast-int-split500-halton.json',
    # 'results/data-int-grid-kinematic.json',
    # 'results/data-int-grid.json',
    # 'results/data-beast-int-halton.json', 
    # 'results/data-beast-int-split.json',
    # 'results/data-int-grid-500.json', 
    'results/data-beats-beatsimp.json', 
    'results/data-beast-int-b1.json') 
    # 'results/data-gust.json',
    # 'results/data-pprm.json',
    # 'results/data-h&q&b-nog.json',
    # 'results/data-dcar&kcar-nog.json')
    # 'results/data-singlewallnarrow-int-g-mod.json', 
    # 'results/data-singlewall-int-g.json') 

print(df.columns)
# df = df.loc[:, ['Domain', 'Algorithm', 'time', 'EnvironmentName']]

add_instance_column(df)
df = calculate_relative_instance_time(df, 'BEAST_INT')

df=df.loc[(df['Domain']!='Quadrotor')&(df['Domain']!='Blimp')]

fig = plt.figure(figsize=(10,12))
count = df.Domain.unique().size
c = int(ceil(sqrt(count)))
r = int(ceil(float(count) / float(c)))
for i, d in enumerate(df.Domain.unique()):
    axcurr = fig.add_subplot(r, c, i + 1)
    box_plot = sns.boxplot(x='EnvironmentName', y='relative_time', 
                           hue='Algorithm', ax = axcurr,
                           data=df.loc[df['Domain'] == d], 
                           showmeans = True, meanline = True, 
                           notch = True, palette = "Set3")
    box_plot.set_title(d)
    max_y_lim = get_lim(df.loc[df['Domain']==d])
    # box_plot.set_ylim(0, max_y_lim)
    box_plot.set_xlabel('')
    box_plot.set_xticklabels(box_plot.get_xticklabels(), rotation = 17)
    if i + 1 == count:
        box_plot.legend(loc =2,  bbox_to_anchor = (1.05, 1))
    else:
        box_plot.legend_.remove()

# plt.tight_layout()
plt.savefig('gust-pprm.eps')
# sns.despine(offset=10, trim=True)
