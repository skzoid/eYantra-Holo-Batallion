#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import re

values = []
values_recalculated =[]
match_prev = 700

with open("data.txt", "r") as f:
    for line in f:
        # Extract only numeric values using regex
        match = line
        match_1 = line
        match_1 = float(match_1)
        match = float(match)
        if abs(match-match_prev) > (5) and match_prev != 700:
            match = match_prev
        match_prev = match
        if match_1:
            values.append(match_1)
        if match:
            values_recalculated.append(match)

values = np.array(values)
values_recalculated = np.array(values_recalculated)

# Drop extra values if not multiple of 3
# n = len(values) // 3
# data = values[:n*3].reshape(n, 3)

# col1 = data[:, 0]
# col2 = data[:, 1]
# col3 = data[:, 2]

# Plot
fig, axs = plt.subplots(3, 1, figsize=(8, 8))

axs[0].plot(values)
axs[0].set_title("Column 1")

axs[1].plot(values_recalculated)
axs[1].set_title("Column 2")

# axs[2].plot(col3)
# axs[2].set_title("Column 3")

plt.tight_layout()
plt.show()
