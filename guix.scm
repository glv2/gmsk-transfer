(use-modules (gnu packages autotools)
             (gnu packages radio)
             (guix build-system gnu)
             (guix git-download)
             ((guix licenses) #:prefix license:)
             (guix packages))

(define gmsk-transfer
  (package
    (name "gmsk-transfer")
    (version "1.1.0")
    (source
     (origin
       (method git-fetch)
       (uri (git-reference
             (url "https://github.com/glv2/gmsk-transfer")
             (commit (string-append "v" version))))
       (file-name (git-file-name name version))
       (sha256
        (base32 "0hspp4mi23i78l1z3n5qrx3gwnn1qiahlh2xdlsn15sp9mikcqqf"))))
    (build-system gnu-build-system)
    (native-inputs
     `(("autoconf" ,autoconf)
       ("automake" ,automake)
       ("libtool" ,libtool)))
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
