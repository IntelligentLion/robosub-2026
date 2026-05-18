# Pull Requests, Conflict Resolution & Git LFS

## Pull Requests and Code Review

### Preparing Pull Requests

```bash
# Create feature branch
git checkout -b feature/add-user-auth

# Make changes and commit
git add .
git commit -m "feat(auth): Add user authentication"

# Push to remote
git push -u origin feature/add-user-auth

# Keep PR branch updated with main
git fetch origin
git rebase origin/main
git push --force-with-lease origin feature/add-user-auth

# Squash commits before PR (interactive rebase)
git rebase -i HEAD~3
git push --force-with-lease origin feature/add-user-auth
```

### Addressing PR Feedback

```bash
# Make requested changes
git add .
git commit -m "fix: Address PR feedback - update validation"

# Or amend last commit
git add .
git commit --amend --no-edit
git push --force-with-lease origin feature/add-user-auth

# Squash all commits in PR
git rebase -i origin/main
# In editor: pick first commit, squash rest
git push --force-with-lease origin feature/add-user-auth
```

---

## Conflict Resolution with Remote

### Handling Merge Conflicts

```bash
# Pull latest changes
git pull origin main
# If conflicts occur:

# 1. View conflicting files
git status

# 2. Resolve conflicts in editor
# Remove conflict markers: <<<<<<<, =======, >>>>>>>

# 3. Mark as resolved
git add conflicted-file.cs

# 4. Complete merge
git commit -m "Merge: Resolve conflicts with main"

# 5. Push
git push origin feature-branch
```

### Rebase Conflicts

```bash
# Start rebase
git rebase origin/main

# If conflicts occur:
# 1. Resolve conflicts
# 2. Stage resolved files
git add .

# 3. Continue rebase
git rebase --continue

# Or abort rebase
git rebase --abort

# Push after successful rebase
git push --force-with-lease origin feature-branch
```

---

## Working with Large Files (Git LFS)

```bash
# Install Git LFS
git lfs install

# Track large file types
git lfs track "*.psd"
git lfs track "*.mp4"
git lfs track "*.zip"

# Commit .gitattributes
git add .gitattributes
git commit -m "Add Git LFS tracking"

# Add large file
git add large-file.zip
git commit -m "Add large binary file"
git push origin main

# Clone repository with LFS files
git clone https://github.com/username/repo.git
cd repo
git lfs pull

# Fetch LFS files for specific commit
git lfs fetch origin main
git lfs checkout
```

---
