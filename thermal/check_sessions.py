
import pandas as pd

df = pd.read_csv("data/scene_features.csv")

features = [
    "mean",
    "std",
    "foreground_coverage",
    "component_count",
    "largest_component_fraction",
    "edge_density"
]

print("\nAverage features by session:")
print(
    df.groupby("scene_id")[features]
    .mean()
    .round(3)
    .to_string()
)

print("\nLabels per session:")
print(
    pd.crosstab(
        df["scene_id"],
        df["subject_type"]
    )
)

print("\nOcclusion per session:")
print(
    pd.crosstab(
        df["scene_id"],
        df["occlusion_level"]
    )
)
