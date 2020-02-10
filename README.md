# Ranger: Regular ArraNGEment of building facade

This is the code corresponding our paper "Fast and Regularized Reconstruction of Building Fa\c{c}ades from Street-View Images using Binary Integer Programming" submitted to ISPRS2020 Congress.

## Requirements
The only requirement is Mosek, which has a free one-year trial version for academic use.

## Usage
`IN_PATH` represents the path of the original box file, `OUT_PATH` represents the output path of the box after regularization.The file suffix is `.xywh`, it means each line in the file represents the coordinates of the upper left corner point, the width of the box and the height of the box. `delta_x`, `delta_y`, `delta_w`, `delta_h` represent the step size of the pre-clustering, `alpha_x`, `alpha_y`, `alpha_w`, `alpha_h` represent each weight, you should change the values of these parameters to get the result you want.

## Example
```cpp
	std::vector<double> x, y, w, h, X, Y, W, H;
	std::vector<int> r;
	auto s1 = read_file(IN_PATH);
	get_xywh(s1, x, y, w, h);
	N = x.size();
	m = X.size();
	n = Y.size();
	m_ = W.size();
	n_ = H.size();
	pre_cluster(x, X, delta_x);
	pre_cluster(y, Y, delta_y);
	pre_cluster(w, W, delta_w);
	pre_cluster(h, H, delta_h);
	regularize(x, y, w, h, X, Y, W, H, r);
	write_file(X, Y, W, H, r, OUT_PATH);
```

## License
MIT License
