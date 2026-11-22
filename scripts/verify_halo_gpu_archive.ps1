param(
    [Parameter(Mandatory=$true)][string]$Archive,
    [Parameter(Mandatory=$true)][string]$Identity,
    [Parameter(Mandatory=$true)][string]$Receipt
)
$ErrorActionPreference = 'Stop'
if (Test-Path -LiteralPath $Receipt) { throw 'Preserve existing verification receipts' }
$archivePath = (Resolve-Path -LiteralPath $Archive).Path
$identityPath = (Resolve-Path -LiteralPath $Identity).Path
$expected = Get-Content -LiteralPath $identityPath -Raw | ConvertFrom-Json
$archiveHash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
if ($archiveHash -cne $expected.sha256 -or (Get-Item -LiteralPath $archivePath).Length -ne $expected.bytes) {
    throw 'Downloaded archive identity mismatch'
}
Add-Type -AssemblyName System.IO.Compression.FileSystem
$zip = [System.IO.Compression.ZipFile]::OpenRead($archivePath)
$verified = [System.Collections.Generic.List[object]]::new()
try {
    $entries = [System.Collections.Generic.Dictionary[string,object]]::new([System.StringComparer]::Ordinal)
    foreach ($entry in $zip.Entries) {
        if ($entries.ContainsKey($entry.FullName)) { throw "Duplicate archive member: $($entry.FullName)" }
        $entries.Add($entry.FullName, $entry)
    }
    if (!$entries.ContainsKey('SHA256.json')) { throw 'Member manifest missing' }
    $reader = [System.IO.StreamReader]::new($entries['SHA256.json'].Open())
    try { $manifest = $reader.ReadToEnd() | ConvertFrom-Json } finally { $reader.Dispose() }
    $properties = @($manifest.PSObject.Properties)
    if ($entries.Count -ne $properties.Count + 1 -or $properties.Count -ne $expected.verified_files) {
        throw 'Archive and manifest member counts differ'
    }
    foreach ($member in $properties) {
        if ($member.Name -ceq 'SHA256.json' -or !$entries.ContainsKey($member.Name)) {
            throw "Invalid manifest member: $($member.Name)"
        }
        $stream = $entries[$member.Name].Open()
        $algorithm = [System.Security.Cryptography.SHA256]::Create()
        try {
            $actual = [System.BitConverter]::ToString($algorithm.ComputeHash($stream)).Replace('-', '').ToLowerInvariant()
        } finally { $algorithm.Dispose(); $stream.Dispose() }
        if ($actual -cne $member.Value) { throw "Member hash mismatch: $($member.Name)" }
        $verified.Add([pscustomobject]@{member=$member.Name; sha256=$actual; bytes=$entries[$member.Name].Length})
    }
} finally { $zip.Dispose() }
$result = [ordered]@{
    utc=[DateTime]::UtcNow.ToString('o'); archive=$archivePath; identity=$identityPath
    archive_sha256=$archiveHash; archive_bytes=(Get-Item -LiteralPath $archivePath).Length
    verified_files=$verified.Count; all_members_verified=$true; members=$verified
}
$result | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $Receipt -Encoding utf8
[pscustomobject]@{archive_sha256=$archiveHash; verified_files=$verified.Count; receipt=$Receipt}
