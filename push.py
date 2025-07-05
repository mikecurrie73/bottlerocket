#!/usr/bin/env python

# ---------------------------------------------------------------------------
# GitHub Push Automation Script
# ---------------------------------------------------------------------------
# This script automates the process of adding, committing, and pushing
# changes to a GitHub repository.
#
# Usage:
# 1. Save this file as 'push.py' in the root of your git repository.
# 2. Run from the terminal with a commit message:
#    python push.py "Your descriptive commit message"
# ---------------------------------------------------------------------------

import sys
import subprocess

def run_command(command):
    """
    Executes a shell command and prints its output in real-time.
    Returns True if the command was successful, False otherwise.
    """
    try:
        # The 'subprocess.run' function is used to execute the command.
        # 'check=True' will raise an exception if the command fails (returns a non-zero exit code).
        # 'shell=True' allows us to pass the command as a single string.
        process = subprocess.run(
            command, 
            shell=True, 
            check=True, 
            text=True, 
            capture_output=True
        )
        # Print the standard output from the command
        if process.stdout:
            print(process.stdout)
        # Print any standard error output
        if process.stderr:
            print(process.stderr)
        return True
    except subprocess.CalledProcessError as e:
        # If the command fails, print the error details.
        print(f"Error executing: '{e.cmd}'")
        print(e.stdout)
        print(e.stderr)
        return False

def main():
    """
    The main function that orchestrates the git commands.
    """
    # --- Step 0: Check for a commit message ---
    # The script requires a commit message as a command-line argument.
    # sys.argv is a list containing the script name and its arguments.
    if len(sys.argv) < 2:
        print("❌ Error: No commit message provided.")
        print("   Usage: python push.py \"Your commit message here\"")
        sys.exit(1) # Exit the script indicating an error

    commit_message = sys.argv[1]

    print("🚀 Starting GitHub push process...")
    
    # --- Step 1: Add all changes ---
    print("\n[Step 1/3] Adding all changed files...")
    if not run_command("git add ."):
        print("❌ Failed to add files. Aborting.")
        sys.exit(1)

    # --- Step 2: Commit the changes ---
    print(f"\n[Step 2/3] Committing with message: \"{commit_message}\"")
    # We construct the commit command. The f-string allows us to embed the message.
    # Note: If there are no changes to commit, this command might "fail", but that's okay.
    # The script will print the message from git and continue.
    run_command(f'git commit -m "{commit_message}"')

    # --- Step 3: Push to GitHub ---
    print("\n[Step 3/3] Pushing to remote repository (origin main)...")
    if not run_command("git push origin main"):
        print("❌ Failed to push to GitHub. Check your connection, credentials, and repository status.")
        sys.exit(1)

    print("\n✅ Success! All changes have been pushed to GitHub.")

if __name__ == "__main__":
    # This ensures the main() function is called only when the script is executed directly.
    main()
#!/usr/bin/env python

# ---------------------------------------------------------------------------
# GitHub Push Automation Script
# ---------------------------------------------------------------------------
# This script automates the process of adding, committing, and pushing
# changes to a GitHub repository.
#
# Usage:
# 1. Save this file as 'push.py' in the root of your git repository.
# 2. Run from the terminal with a commit message:
#    python push.py "Your descriptive commit message"
# ---------------------------------------------------------------------------

import sys
import subprocess

def run_command(command):
    """
    Executes a shell command and prints its output in real-time.
    Returns True if the command was successful, False otherwise.
    """
    try:
        # The 'subprocess.run' function is used to execute the command.
        # 'check=True' will raise an exception if the command fails (returns a non-zero exit code).
        # 'shell=True' allows us to pass the command as a single string.
        process = subprocess.run(
            command, 
            shell=True, 
            check=True, 
            text=True, 
            capture_output=True
        )
        # Print the standard output from the command
        if process.stdout:
            print(process.stdout)
        # Print any standard error output
        if process.stderr:
            print(process.stderr)
        return True
    except subprocess.CalledProcessError as e:
        # If the command fails, print the error details.
        print(f"Error executing: '{e.cmd}'")
        print(e.stdout)
        print(e.stderr)
        return False

def main():
    """
    The main function that orchestrates the git commands.
    """
    # --- Step 0: Check for a commit message ---
    # The script requires a commit message as a command-line argument.
    # sys.argv is a list containing the script name and its arguments.
    if len(sys.argv) < 2:
        print("❌ Error: No commit message provided.")
        print("   Usage: python push.py \"Your commit message here\"")
        sys.exit(1) # Exit the script indicating an error

    commit_message = sys.argv[1]

    print("🚀 Starting GitHub push process...")
    
    # --- Step 1: Add all changes ---
    print("\n[Step 1/3] Adding all changed files...")
    if not run_command("git add ."):
        print("❌ Failed to add files. Aborting.")
        sys.exit(1)

    # --- Step 2: Commit the changes ---
    print(f"\n[Step 2/3] Committing with message: \"{commit_message}\"")
    # We construct the commit command. The f-string allows us to embed the message.
    # Note: If there are no changes to commit, this command might "fail", but that's okay.
    # The script will print the message from git and continue.
    run_command(f'git commit -m "{commit_message}"')

    # --- Step 3: Push to GitHub ---
    print("\n[Step 3/3] Pushing to remote repository (origin main)...")
    if not run_command("git push origin main"):
        print("❌ Failed to push to GitHub. Check your connection, credentials, and repository status.")
        sys.exit(1)

    print("\n✅ Success! All changes have been pushed to GitHub.")

if __name__ == "__main__":
    # This ensures the main() function is called only when the script is executed directly.
    main()

