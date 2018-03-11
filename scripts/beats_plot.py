import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

df = pd.read_json('data-latest.json')
print(df.columns)
df = df.loc[:, ['Planner', 'time', 'EnvironmentMesh']]
print(df.Planner.unique())

sns.boxplot(x='EnvironmentMesh', y='time', hue='Planner', data=df)

# plt.show()
plt.savefig('test.svg')
# sns.despine(offset=10, trim=True)
