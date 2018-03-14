import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

df = pd.read_json('data-latest.json')
print(df.columns)
# df = df.loc[:, ['Planner', 'time', 'EnvironmentMesh']]
print(df.Planner.unique())

df['instance'] = df.Domain + '_' + df.Seed.astype(str) + '_' + \
                 df.AgentMesh + '_' + df.EnvironmentMesh + '_' + \
                 df.EnvironmentBounds + '_' + df.Start + '_' + df.Goal

df.instance = df.instance.str.replace(' ', '#')

sns.boxplot(x='EnvironmentMesh', y='time', hue='Planner', data=df)
plt.show()
# plt.savefig('test.svg')

df.pivot(index='instance', columns='Planner')['time'].plot.scatter('BEAST', 'BEATS')
plt.savefig('test.svg')
plt.show()

if __name__ == '__main__':
    print('done')
