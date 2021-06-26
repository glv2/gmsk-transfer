(use-modules (gnu packages autotools)
             (gnu packages radio)
             (guix build-system gnu)
             (guix git-download)
             ((guix licenses) #:prefix license:)
             (guix packages))

(define gmsk-transfer
  (package
    (name "gmsk-transfer")
    (version "1.0.0")
    (source
     (origin
       (method git-fetch)
       (uri (git-reference
             (url "https://github.com/glv2/gmsk-transfer")
             (commit (string-append "v" version))))
       (file-name (git-file-name name version))
       (sha256
        (base32 "1w79i1gvr5f0wjwcwkh7kj4pw1758rxd1lzkchfk0q057cciv15m"))))
    (build-system gnu-build-system)
    (native-inputs
     `(("autoconf" ,autoconf)
       ("automake" ,automake)))
    (inputs
     `(("liquid-dsp" ,liquid-dsp)
       ("soapysdr" ,soapysdr)))
    (synopsis "Program to transfer data by radio")
    (description
     "@code{gmsk-transfer} is a command-line program to send or receive data by
radio using the GMSK modulation.")
    (home-page "https://github.com/glv2/gmsk-transfer")
    (license license:gpl3+)))

gmsk-transfer
