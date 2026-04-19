# PRiMEStereoMatch

**Please use these citations in your publication if you use this work:** ([bibtex here](https://github.com/PRiME-project/PRiMEStereoMatch#license))  

Charles Leech, Charan Kumar, Amit Acharyya, Sheng Yang, Geoff V. Merrett, and Bashir M. Al-Hashimi. 2017. Runtime Performance and Power Optimization of Parallel Disparity Estimation on Many-Core Platforms. ACM Transactions on Embedded Computing Systems (TECS) Volume 17 Issue 2, Article 41 (November 2017), 19 pages. DOI: https://doi.org/10.1145/3133560  

Leech, Charles (2018) [Runtime energy management of multi-core processors.](https://eprints.soton.ac.uk/422287/) University of Southampton, Doctoral Thesis, 293pp. 

---
<p align="center">
<img src="docs/de_examples.png" alt="Examples Image Pairs" width=80%>
</p>

## Theoretical Background

A heterogeneous and fully parallel stereo matching algorithm for depth estimation. Stereo Matching is based on the disparity estimation algorithm, an algorithm designed to calculate 3D depth information about a scene from a pair of 2D images captured by a stereoscopic camera. The algorithm contains the following stages:

* Cost Volume Construction - weighted absolute difference of colours and gradients function.
* Cost Volume Filtering - Adaptive Support Weight (ADSW) Guided Image Filter (GIF) function.  
* Disparity Selection - Winner-Takes-All (WTA) minimum cost selection.  
* Post Processing - left-right occlusion check, invalid pixel replacement and weight-median filtering.  

<p align="center">
<img src="docs/de_bd.png" alt="Disparity estimation process block diagram" width=80%>
</p>

## Implementation Details

C++ parallelism is introduced via the POSIX threads (pthreads) library.
Disparity level parallelism is supported, enabling up to 64 concurrent threads.

## Prerequisites
 * OpenCV 4.0 or later
 * cmake v3.20 or later
 * openMP

## Compilation and installation

 * `cmake .`
 * `make`
 * `sudo make install`

## Demo application

The [demo](demo) directory contains an example application how to detect disparity.

Run the application with: `/PRiMEStereoMatch [[left.png right.png] disparity.png]`

## References

### Code

Some components of the application are based on source code from the following locations:

 [rookiepig/CrossScaleStereo](https://github.com/rookiepig/CrossScaleStereo) - The basis for some C++ functions (GNU Public License)

 [atilimcetin/guided-filter](https://github.com/atilimcetin/guided-filter) - CPU-based GIF implementation using the Fast Guided Filter (MIT License)

### Literature

The algorithm in this work is based in parts on those presented in the following publications:  

<a name="Hosni2011CVPR">[Hosni2011CVPR]</a>: C. Rhemann, A. Hosni, M. Bleyer, C. Rother, and M. Gelautz. Fast cost-volume filtering for visual correspondence and beyond. In CVPR, 2011

<a name="Hosni2011ICME">[Hosni2011ICME]</a>: A. Hosni, M. Bleyer, C. Rhemann, M. Gelautz and C. Rother, Real-time local stereo matching using guided image filtering, in Multimedia and Expo (ICME), 2011 IEEE International Conference on, Barcelona, 2011. 

<a name="Ttofis2014">[Ttofis2014]</a>: C. Ttofis and T. Theocharides, High-quality real-time hardware stereo matching based on guided image filtering, in Design, Automation and Test in Europe Conference and Exhibition (DATE), Dresden, 2014. 

<a name="He2012">[He2012]</a>: K. He, J. Sun and X. Tang, Guided Image Filtering, Pattern Analysis and Machine Intelligence, IEEE Transactions on, pp. 1397-1409, 02 October 2012. 

## License

### Orig work

This software is released under the BSD 3 Clause License. See LICENSE.txt for details.

To cite this code in your work, please also include the following reference:

Charles Leech, Charan Kumar, Amit Acharyya, Sheng Yang, Geoff V. Merrett, and Bashir M. Al-Hashimi. 2017. Runtime Performance and Power Optimization of Parallel Disparity Estimation on Many-Core Platforms. ACM Transactions on Embedded Computing Systems (TECS) Volume 17 Issue 2, Article 41 (November 2017), 19 pages. DOI: https://doi.org/10.1145/3133560 

Bibtex:
```
@article{Leech:2017:RPP:3160927.3133560,
 author = {Leech, Charles and Kumar, Charan and Acharyya, Amit and Yang, Sheng and Merrett, Geoff V. and Al-Hashimi, Bashir M.},
 title = {Runtime Performance and Power Optimization of Parallel Disparity Estimation on Many-Core Platforms},
 journal = {ACM Transactions on Embedded Computing Systems (TECS)},
 issue_date = {January 2018},
 volume = {17},
 number = {2},
 month = nov,
 year = {2017},
 issn = {1539-9087},
 pages = {41:1--41:19},
 articleno = {41},
 numpages = {19},
 url = {http://doi.acm.org/10.1145/3133560},
 doi = {10.1145/3133560},
 acmid = {3133560},
 publisher = {ACM},
 address = {New York, NY, USA},
 keywords = {Runtime management, computer vision, many-core platforms, power optimization},
} 
```

### Further development

Updated by Bernd Porr for openCV 4.0 and turned into a library.

