using GLMakie

range_image_file = "range_image.dat"
f = open(range_image_file, "r")
values = strip(read(f, String))
close(f)

parsed_values = reshape(collect(Iterators.map(s -> parse(Float64, s), split(values, " "))), 88, 58)
println(size(parsed_values))

fig, ax, hm = heatmap(parsed_values)
Colorbar(fig[:, end+1], hm)
fig
