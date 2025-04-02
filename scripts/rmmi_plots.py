#  name,solved,IK_total,IK_avg,plan_tries,wall_time

import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt


data = pd.read_csv("bookshelf_results.csv")

# Plot wall time
sns.boxplot(data["wall_time"], log_scale = True)
plt.ylabel("time [s]")
plt.savefig("Bookshelf_walltime.png")

plt.cla()


# Plot IK tries
sns.boxplot(data["IK_total"])
plt.ylabel("total IK sampled")
plt.savefig("Bookshelf_IK_Total.png")



