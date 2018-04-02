import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from math import * 


df4 = pd.read_json('data-hnb-kcar&dcar-nog.json')
df2 = pd.read_json('data-hnb-h&q&b-nog.json')
df3 = pd.read_json('data-h&q&b-nog.json')
df5 = pd.read_json('data-beast-int-full.json')
df6 = pd.read_json('data-latest.json')
df = pd.read_json('data-dcar&kcar-nog.json')
df = df.append(df2)
df = df.append(df4)
df = df.append(df3)
df = df.append(df5)
df = df.append(df6)
#df = pd.read_json('data-23-57-31-03-18.json')
#df2 = pd.read_json('data-latest.json')
#df=df.append(df2)
print(df.columns)
df = df.loc[:, ['Domain', 'Algorithm', 'time', 'EnvironmentName']]

fig = plt.figure()
count = df.Domain.unique().size
c = int(ceil(sqrt(count)))
r = int(ceil(float(count) / float(c)))
#sns.set(font_scale = 0.8)
for i, d in enumerate(df.Domain.unique()):
    axcurr = fig.add_subplot(r, c, i + 1)
    box_plot = sns.boxplot(x='EnvironmentName', y='time', \
                           hue='Algorithm', ax = axcurr,\
                           data=df.loc[df['Domain'] == d], \
                           showmeans = True, meanline = True, \
                           notch = True, palette = "Set3")
    box_plot.set_title(d)
    #box_plot.set_ylim(0,df.loc[(df['Domain']==d)&(df['Algorithm']=='BEASTSLOW')].time.quantile(.8))
    box_plot.set_xlabel('')
    box_plot.set_xticklabels(box_plot.get_xticklabels(), rotation = 17)
    if i + 1 == count:
        box_plot.legend(loc =2,  bbox_to_anchor = (1.05, 1))
    else:
        box_plot.legend_.remove()

plt.tight_layout()
plt.savefig('int-vs-beast-01.eps')
# sns.despine(offset=10, trim=True)
