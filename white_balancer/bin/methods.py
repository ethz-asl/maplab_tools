import numpy as np
import cv2
import math

def stretch_pre(nimg):
    """
    from 'Applicability Of White-Balancing Algorithms to Restoring Faded Colour Slides: An Empirical Evaluation'
    """
    nimg = nimg.transpose(2, 0, 1)
    nimg[0] = np.maximum(nimg[0]-nimg[0].min(),0)
    nimg[1] = np.maximum(nimg[1]-nimg[1].min(),0)
    nimg[2] = np.maximum(nimg[2]-nimg[2].min(),0)
    return nimg.transpose(1, 2, 0)

def retinex(nimg):
    nimg = nimg[..., ::-1]  # convert from BGR to RGB
    nimg = nimg.transpose(2, 0, 1).astype(np.uint32)
    mu_g = nimg[1].max()
    nimg[0] = np.minimum(nimg[0]*(mu_g/float(nimg[0].max())),255)
    nimg[2] = np.minimum(nimg[2]*(mu_g/float(nimg[2].max())),255)
    return nimg.transpose(1, 2, 0).astype(np.uint8)


def max_white(nimg):
    nimg = nimg[..., ::-1]  # convert from BGR to RGB
    if nimg.dtype==np.uint8:
        brightest=float(2**8)
    elif nimg.dtype==np.uint16:
        brightest=float(2**16)
    elif nimg.dtype==np.uint32:
        brightest=float(2**32)
    else:
        brightest==float(2**8)
    nimg = nimg.transpose(2, 0, 1)
    nimg = nimg.astype(np.int32)
    nimg[0] = np.minimum(nimg[0] * (brightest/float(nimg[0].max())),255)
    nimg[1] = np.minimum(nimg[1] * (brightest/float(nimg[1].max())),255)
    nimg[2] = np.minimum(nimg[2] * (brightest/float(nimg[2].max())),255)
    return nimg.transpose(1, 2, 0).astype(np.uint8)

def stretch(nimg):
    return max_white(stretch_pre(nimg))

def retinex_adjust(nimg):
    """
    from 'Combining Gray World and Retinex Theory for Automatic White Balance in Digital Photography'
    """
    nimg = nimg.transpose(2, 0, 1).astype(np.uint32)
    sum_r = np.sum(nimg[0])
    sum_r2 = np.sum(nimg[0]**2)
    max_r = nimg[0].max()
    max_r2 = max_r**2
    sum_g = np.sum(nimg[1])
    max_g = nimg[1].max()
    coefficient = np.linalg.solve(np.array([[sum_r2,sum_r],[max_r2,max_r]]),
                                  np.array([sum_g,max_g]))
    nimg[0] = np.minimum((nimg[0]**2)*coefficient[0] + nimg[0]*coefficient[1],255)
    sum_b = np.sum(nimg[1])
    sum_b2 = np.sum(nimg[1]**2)
    max_b = nimg[1].max()
    max_b2 = max_r**2
    coefficient = np.linalg.solve(np.array([[sum_b2,sum_b],[max_b2,max_b]]),
                                             np.array([sum_g,max_g]))
    nimg[1] = np.minimum((nimg[1]**2)*coefficient[0] + nimg[1]*coefficient[1],255)
    return nimg.transpose(1, 2, 0).astype(np.uint8)

def retinex_with_adjust(nimg):
    return retinex_adjust(retinex(nimg))

def simplest_cb(img, percent):
    img = img[..., ::-1]  # convert from BGR to RGB
    out_channels = []
    channels = cv2.split(img)
    totalstop = channels[0].shape[0] * channels[0].shape[1] * percent / 200.0
    for channel in channels:
        bc = cv2.calcHist([channel], [0], None, [256], (0,256), accumulate=False)
        lv = np.searchsorted(np.cumsum(bc), totalstop)
        hv = 255-np.searchsorted(np.cumsum(bc[::-1]), totalstop)
        lut = np.array([0 if i < lv else (255 if i > hv else round(float(i-lv)/float(hv-lv)*255)) for i in np.arange(0, 256)], dtype="uint8")
        out_channels.append(cv2.LUT(channel, lut))
    return cv2.merge(out_channels)

def simplest_cb2(img, percent=1):
    img = img[..., ::-1]  # convert from BGR to RGB
    out_channels = []
    cumstops = (
        img.shape[0] * img.shape[1] * percent / 200.0,
        img.shape[0] * img.shape[1] * (1 - percent / 200.0)
    )
    for channel in cv2.split(img):
        cumhist = np.cumsum(cv2.calcHist([channel], [0], None, [256], (0,256)))
        low_cut, high_cut = np.searchsorted(cumhist, cumstops)
        lut = np.concatenate((
            np.zeros(low_cut),
            np.around(np.linspace(0, 255, high_cut - low_cut + 1)),
            255 * np.ones(255 - high_cut)
        ))
        out_channels.append(cv2.LUT(channel, lut.astype('uint8')))
    return cv2.merge(out_channels)

def color_balance(img):
    img = img[..., ::-1]  # convert from BGR to RGB
    x = []

    for i in cv2.split(img):
        hist, bins = np.histogram(i, 256, (0, 256))
        # discard colors at each end of the histogram which are used by only 0.05%
        tmp = np.where(hist > hist.sum() * 0.0005)[0]
        i_min = tmp.min()
        i_max = tmp.max()
        # stretch hist
        tmp = (i.astype(np.int32) - i_min) / (i_max - i_min) * 255
        tmp = np.clip(tmp, 0, 255)
        x.append(tmp.astype(np.uint8))

    # combine image back and show it
    s = np.dstack(x)
    return s


def color_correction_of_image_analysis(img):
    def detection(img):
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(img_lab)
        d_a, d_b, M_a, M_b = 0, 0, 0, 0
        for i in range(m):
            for j in range(n):
                d_a = d_a + a[i][j]
                d_b = d_b + b[i][j]
        d_a, d_b = (d_a / (m * n)) - 128, (d_b / (n * m)) - 128
        D = np.sqrt((np.square(d_a) + np.square(d_b)))

        for i in range(m):
            for j in range(n):
                M_a = np.abs(a[i][j] - d_a - 128) + M_a
                M_b = np.abs(b[i][j] - d_b - 128) + M_b

        M_a, M_b = M_a / (m * n), M_b / (m * n)
        M = np.sqrt((np.square(M_a) + np.square(M_b)))
        k = D / M
        return

    r, g, b = cv2.split(img)
    m, n = b.shape

    I_r_2 = np.zeros(r.shape)
    I_b_2 = np.zeros(b.shape)

    I_r_2 = (r.astype(np.float32) ** 2).astype(np.float32)
    I_b_2 = (b.astype(np.float32) ** 2).astype(np.float32)
    sum_I_r_2 = I_r_2.sum()
    sum_I_b_2 = I_b_2.sum()
    sum_I_g = g.sum()
    sum_I_r = r.sum()
    sum_I_b = b.sum()

    max_I_r = r.max()
    max_I_g = g.max()
    max_I_b = b.max()
    max_I_r_2 = I_r_2.max()
    max_I_b_2 = I_b_2.max()

    [u_b, v_b] = np.matmul(np.linalg.inv([[sum_I_b_2, sum_I_b], [max_I_b_2, max_I_b]]), [sum_I_g, max_I_g])
    [u_r, v_r] = np.matmul(np.linalg.inv([[sum_I_r_2, sum_I_r], [max_I_r_2, max_I_r]]), [sum_I_g, max_I_g])

    b_point = u_b * (b.astype(np.float32) ** 2) + v_b * b.astype(np.float32)
    r_point = u_r * (r.astype(np.float32) ** 2) + v_r * r.astype(np.float32)

    b = np.clip(b_point, 0, 255).astype(np.uint8)
    r = np.clip(r_point, 0, 255).astype(np.uint8)

    return cv2.merge([b, g, r])


def perfect_reflective_white_balance(img):
    img = img[..., ::-1]  # convert from BGR to RGB
    b, g, r = cv2.split(img)
    m, n, t = img.shape
    sum_ = np.zeros(b.shape)
    sum_ = b.astype(np.int32) + g.astype(np.int32) + r.astype(np.int32)

    hists, bins = np.histogram(sum_.flatten(), 766, [0, 766])
    Y = 765
    num, key = 0, 0
    ratio = 0.01
    while Y >= 0:
        num += hists[Y]
        if num > m * n * ratio / 100:
            key = Y
            break
        Y = Y - 1

    sum_b = b[sum_ >= key].sum()
    sum_g = g[sum_ >= key].sum()
    sum_r = r[sum_ >= key].sum()
    time = (sum_ >= key).sum()

    avg_b = sum_b / time
    avg_g = sum_g / time
    avg_r = sum_r / time

    maxvalue = float(np.max(img))

    b = img[:, :, 0].astype(np.int32) * maxvalue / int(avg_b)
    g = img[:, :, 1].astype(np.int32) * maxvalue / int(avg_g)
    r = img[:, :, 2].astype(np.int32) * maxvalue / int(avg_r)
    b[b > 255] = 255
    b[b < 0] = 0
    g[g > 255] = 255
    g[g < 0] = 0
    r[r > 255] = 255
    r[r < 0] = 0
    img[:, :, 0] = b
    img[:, :, 1] = g
    img[:, :, 2] = r

    return img


def gamma_trans(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mean = np.mean(img_gray)
    gamma = math.log10(0.5) / math.log10(mean / 255) # The formula calculates gamma
    gamma_table = [np.power(x / 255.0, gamma) * 255.0 for x in range(256)] # Create a mapping table
    gamma_table = np.round(np.array(gamma_table)).astype(np.uint8) # The color value is an integer
    return cv2.LUT(img, gamma_table) # Look up the table of image colors. In addition, an adaptive algorithm can be designed according to the principle of light intensity (color) uniformity.
