import pandas as pd
import joblib
import argparse
import numpy as np

from scene_features import DATA, FEATURES

from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import LeaveOneGroupOut
from sklearn.metrics import (
    accuracy_score,
    balanced_accuracy_score,
    brier_score_loss,
    classification_report,
    confusion_matrix,
    roc_auc_score,
)

parser = argparse.ArgumentParser(description="Evaluate and train the experimental thermal scene model")
parser.add_argument("--evaluate-only", action="store_true")
args = parser.parse_args()
df = pd.read_csv(DATA / "scene_features.csv")
df = df.loc[df["subject_type"].isin(["empty", "human", "nonhuman"])].reset_index(drop=True)
if df["session_id"].isna().any() or df["session_id"].nunique() < 2:
    parser.exit(1, "Evaluation requires at least two identified recording sessions.\n")

X = df[FEATURES]
# The live demo needs one answer: how likely is a person present?
y = (df["subject_type"] == "human").astype(int)
groups = df["session_id"]

print("Images:", len(df))
print("Target: human present vs no human")
print("Sessions:", groups.nunique())

# Evaluate on recording sessions not used in training.
logo = LeaveOneGroupOut()
out_of_fold = np.empty(len(df), dtype=int)
majority = np.empty(len(df), dtype=int)
out_of_fold_probability = np.zeros(len(df), dtype=float)

for fold, (train_idx, test_idx) in enumerate(
    logo.split(X, y, groups), start=1
):
    model = RandomForestClassifier(
        n_estimators=300,
        max_depth=4,
        min_samples_leaf=5,
        class_weight="balanced",
        random_state=42
    )

    model.fit(X.iloc[train_idx], y.iloc[train_idx])

    predictions = model.predict(X.iloc[test_idx])
    human_index = list(model.classes_).index(1)
    out_of_fold_probability[test_idx] = model.predict_proba(
        X.iloc[test_idx]
    )[:, human_index]
    actual = y.iloc[test_idx]
    out_of_fold[test_idx] = predictions
    majority[test_idx] = y.iloc[train_idx].mode().iloc[0]
    print(f"\n===== TEST SESSION {fold} =====")
    print("Session:", groups.iloc[test_idx].iloc[0])
    print("Test images:", len(test_idx))

    print(
        "Accuracy:",
        round(accuracy_score(actual, predictions), 3)
    )
    print(classification_report(
        actual,
        predictions,
        target_names=["no human", "human"],
        zero_division=0
    ))

    print("Confusion matrix:")
    print("Labels: no human, human")
    print(confusion_matrix(
        actual, predictions, labels=[0, 1]
    ))

print("\n===== POOLED HELD-OUT SESSION RESULTS =====")
print(f"Accuracy: {accuracy_score(y, out_of_fold):.1%}")
print(f"Balanced accuracy: {balanced_accuracy_score(y, out_of_fold):.1%}")
print(f"ROC AUC: {roc_auc_score(y, out_of_fold_probability):.1%}")
print(f"Brier score: {brier_score_loss(y, out_of_fold_probability):.3f} (lower is better)")
print(f"Training-majority baseline accuracy: {accuracy_score(y, majority):.1%}")
print(classification_report(
    y,
    out_of_fold,
    labels=[0, 1],
    target_names=["no human", "human"],
    zero_division=0,
))
if args.evaluate_only:
    raise SystemExit(0)

# Train a final prototype on all available images.
final_model = RandomForestClassifier(
    n_estimators=300,
    max_depth=4,
    min_samples_leaf=5,
    class_weight="balanced",
    random_state=42
)

final_model.fit(X, y)

artifact = {
    "model": final_model,
    "features": FEATURES,
    "target": "human_present",
    "held_out_accuracy": float(accuracy_score(y, out_of_fold)),
    "held_out_balanced_accuracy": float(balanced_accuracy_score(y, out_of_fold)),
    "held_out_roc_auc": float(roc_auc_score(y, out_of_fold_probability)),
    "held_out_brier_score": float(brier_score_loss(y, out_of_fold_probability)),
}

joblib.dump(artifact, DATA / "thermal_model.joblib")

print("\nModel saved to data/thermal_model.joblib")

print("\nFeature importance:")
for name, importance in sorted(
    zip(FEATURES, final_model.feature_importances_),
    key=lambda item: item[1],
    reverse=True
):
    print(f"{name}: {importance:.3f}")
