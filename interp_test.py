import numpy as np

def cosine_interp_list(a, b, num_points):
    lst = []
    for i in range(len(a)):
        lst.append(cosine_interp_vals(a[i], b[i], num_points))
    return lst


def cosine_interp_vals(a, b, num_points):
    t = np.linspace(0, 1, num_points)
    # Transform t to the cosine space
    t = (1 - np.cos(t * np.pi)) / 2
    return [(1-tt) * a + tt * b for tt in t]


# Example usage
list1 = [1]
list2 = [5]
num_points = 15  # Number of interpolation steps between the lists

interpolated_values = cosine_interp_list(list1, list2, num_points)
print(interpolated_values)
# for i, step in enumerate(interpolated_values):
#     print(f"Step {i+1}: {step}")
