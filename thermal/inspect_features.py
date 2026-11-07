
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

print("\nFeature statistics by subject label:")
print(
    df.groupby("subject_type")[features]
    .mean()
    .round(3)
    .to_string()
)

print("\nFeature statistics by occlusion:")
print(
    df.groupby("occlusion_level")[features]
    .mean()
    .round(3)
    .to_string()
)

print("\nSession distribution:")
print(
    pd.crosstab(
        df["session_id"],
        df["subject_type"]
    )
)
