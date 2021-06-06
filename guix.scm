(use-modules (gnu packages autotools)
             (gnu packages radio)
             (guix build-system gnu)
             ((guix licenses) #:prefix license:)
             (guix packages))

(define gmsk-transfer
  (package
    (name "gmsk-transfer")
    (version "1.0.0")
    (source #f)
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
    (home-page #f)
    (license license:gpl3+)))

gmsk-transfer
