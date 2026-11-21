puts "Installed families:"
set parts [get_parts *7a35t*]
puts "Artix 35t parts: $parts"
if {[llength $parts] == 0} {
    set parts [get_parts -filter {FAMILY == "artix7"}]
    puts "Artix-7 parts: [lrange $parts 0 5]"
}
if {[llength $parts] == 0} {
    set all_parts [get_parts]
    puts "Sample available parts: [lrange $all_parts 0 5]"
}
exit
