# Define variables
$remoteHost  = "euler"  # The hostname or IP of the remote server
$remoteBase  = "/srv/home/fang/wavemaker/build_wisc/bin/"
$localBase   = "C:\Users\fang\Documents\NREL_WATER\post_processing\data_Drop\"  # Change this to wherever you want to store files locally.
$run_tag = "new_mass"
# variable for clean the script or not in the localBase directory
$clean = $true

# Make sure local base directory exists
if (-not (Test-Path $localBase)) {
    New-Item -ItemType Directory -Path $localBase | Out-Null
}

# Get the list of remote folders containing "various_d0"
# This will execute 'ls' on the remote host, filter for "various_d0", and return the directory names.
$dirList = ssh $remoteHost "ls $remoteBase | grep $run_tag" 2>$null

if ([string]::IsNullOrWhiteSpace($dirList)) {
    Write-Host "No directories found or failed to connect to remote host."
    exit 1
}

# let's print out the dirList
Write-Host "Remote directories:"
Write-Host $dirList

# Filter folders to only include those starting with "ObjectDrop"
$folders = $dirList -split "`n" | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne "" -and $_ -match "^ObjectDrop" }

Write-Host "Found $($folders.Count) ObjectDrop folders to process..."

foreach ($folder in $folders) {
    # Construct remote and local paths
    $remoteFile_fsi = "$remoteBase/$folder/fsi/FSI_body0.csv"
    $localFolder = Join-Path $localBase $folder

    # Create local folder if it doesn't exist
    if (-not (Test-Path $localFolder)) {
        New-Item -ItemType Directory -Path $localFolder | Out-Null
    }

    # Copy the file using scp
    Write-Host "Copying $remoteFile from $remoteHost to $localFile..."
    scp "$($remoteHost):$remoteFile" $localFile

    # Copy the remotFile_fsi using scp, and save it to $localFolder_fsi
    Write-Host "Copying $remoteFile_fsi from $remoteHost to $localFolder ..."
    scp "$($remoteHost):$remoteFile_fsi" $localFolder
}


if ($clean) {

# Get all directories in the base folder
    Get-ChildItem -Path $localBase -Directory | ForEach-Object {
        $oldName = $_.Name
        if ($oldName -match "^(.*)_\d{7}$") {
            $newName = $Matches[1]
            $oldPath = Join-Path $localBase $oldName
            $newPath = Join-Path $localBase $newName
            if (-not (Test-Path $newPath)) {
                Rename-Item -Path $oldPath -NewName $newName
                Write-Output "Renamed: $oldName -> $newName"
            } else {
                # if alredy exists, remove the old one and rename the new one 
                Remove-Item -Path $newPath
                Rename-Item -Path $oldPath -NewName $newName
                Write-Output "Renamed: $oldPath -> $newPath"
            }
        }
    }
}